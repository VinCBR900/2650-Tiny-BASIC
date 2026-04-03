/* ============================================================================
 * sim2650.v
 * Behavioral Verilog port of sim2650.c (v1.7 semantics)
 *
 * Notes:
 *   - This is a software-style simulator written in Verilog for use under a
 *     simulator (e.g. Icarus/Verilator), not a synthesizable CPU core.
 *   - Memory image loading uses $readmemh via +hex=<file>. The input file is
 *     expected to be a plain hex memory file indexed from address 0.
 *   - Optional runtime controls via plusargs:
 *       +trace                 Enable per-instruction trace.
 *       +entry=<hexaddr>       Entry point (default 0x0000; 0x0440 with +pipbug)
 *       +break=<hexaddr>       Breakpoint address
 *       +maxinstr=<decimal>    Instruction limit (default 5000000)
 *       +pipbug                Enable PIPBUG map/intercepts
 * ============================================================================ */

`timescale 1ns/1ps

module sim2650;
  localparam SIM_VER   = "1.7";
  localparam MEM_SIZE  = 16'h8000;

  localparam DEF_ROM_START = 16'h0000;
  localparam DEF_ROM_END   = 16'h13FF;
  localparam DEF_RAM_START = 16'h1400;
  localparam DEF_RAM_END   = 16'h1BFF;

  localparam PB_ROM_START  = 16'h0000;
  localparam PB_ROM_END    = 16'h03FF;
  localparam PB_RAM_START  = 16'h0400;
  localparam PB_RAM_END    = 16'h1BFF;

  localparam PSU_S   = 8'h80;
  localparam PSU_F   = 8'h40;
  localparam PSU_II  = 8'h20;
  localparam PSU_SP  = 8'h07;

  localparam PSL_CC  = 8'hC0;
  localparam PSL_IDC = 8'h20;
  localparam PSL_RS  = 8'h10;
  localparam PSL_WC  = 8'h08;
  localparam PSL_OVF = 8'h04;
  localparam PSL_COM = 8'h02;
  localparam PSL_C   = 8'h01;

  localparam CC_NEG  = 8'h80;
  localparam CC_POS  = 8'h40;
  localparam CC_ZERO = 8'h00;

  localparam COND_EQ = 2'd0;
  localparam COND_GT = 2'd1;
  localparam COND_LT = 2'd2;

  reg [7:0] mem [0:MEM_SIZE-1];
  reg [7:0] R [0:7];
  reg [14:0] RAS [0:7];

  reg [14:0] IAR;
  reg [7:0] PSU;
  reg [7:0] PSL;
  integer SP;

  reg [15:0] ROM_START_A, ROM_END_A, RAM_START_A, RAM_END_A;
  integer trace, running, breakpt, mem_warn_count, run_fault, use_pipbug;
  integer icount, maxinstr;
  reg [14:0] entry_point;

  integer i;
  integer fd;
  reg [1023:0] hexfile;
  reg [1023:0] tmpstr;

  function integer ri;
    input integer n;
    begin
      if (n == 0) ri = 0;
      else ri = (PSL & PSL_RS) ? (n + 3) : n;
    end
  endfunction

  function integer addr_ok;
    input [14:0] a;
    begin
      addr_ok = ((a >= ROM_START_A[14:0]) && (a <= ROM_END_A[14:0])) ||
                ((a >= RAM_START_A[14:0]) && (a <= RAM_END_A[14:0]));
    end
  endfunction

  function [7:0] mrd;
    input [14:0] a;
    begin
      if (!addr_ok(a)) begin
        if (mem_warn_count < 16) $display("WARN: unmap rd $%04h", a);
        mem_warn_count = mem_warn_count + 1;
        mrd = 8'hFF;
      end else begin
        mrd = mem[a];
      end
    end
  endfunction

  task mwr;
    input [14:0] a;
    input [7:0] v;
    begin
      if (!addr_ok(a)) begin
        if (mem_warn_count < 16) $display("WARN: unmap wr $%04h", a);
        mem_warn_count = mem_warn_count + 1;
      end else if ((a >= ROM_START_A[14:0]) && (a <= ROM_END_A[14:0])) begin
        $display("WARN: ROM wr $%04h ignored", a);
      end else begin
        mem[a] = v;
      end
    end
  endtask

  task fetch;
    output [7:0] b;
    begin
      b = mrd(IAR);
      IAR = (IAR + 1) & 15'h7FFF;
    end
  endtask

  task set_cc;
    input [7:0] r;
    begin
      PSL = PSL & ~PSL_CC;
      if (r == 8'h00) PSL = PSL | CC_ZERO;
      else if ($signed(r) > 0) PSL = PSL | CC_POS;
      else PSL = PSL | CC_NEG;
    end
  endtask

  task set_cc_add;
    input [7:0] r;
    begin
      if (!(PSL & PSL_C)) PSL = (PSL & ~PSL_CC) | CC_POS;
      else if (r == 8'h00) PSL = (PSL & ~PSL_CC) | CC_ZERO;
      else PSL = (PSL & ~PSL_CC) | CC_NEG;
    end
  endtask

  task set_cc_sub;
    input [7:0] r;
    begin
      if ((PSL & PSL_C) && (r == 8'h00)) PSL = (PSL & ~PSL_CC) | CC_ZERO;
      else if (PSL & PSL_C) PSL = (PSL & ~PSL_CC) | CC_POS;
      else PSL = (PSL & ~PSL_CC) | CC_NEG;
    end
  endtask

  function integer test_cc;
    input [1:0] cond;
    reg [7:0] cc;
    begin
      cc = PSL & PSL_CC;
      case (cond)
        COND_EQ: test_cc = (cc == CC_ZERO);
        COND_GT: test_cc = (cc == CC_POS);
        COND_LT: test_cc = (cc == CC_NEG);
        default: test_cc = 1;
      endcase
    end
  endfunction

  task push_ras;
    input [14:0] a;
    begin
      SP = (SP + 1) & 7;
      RAS[SP] = a;
      PSU = (PSU & ~PSU_SP) | (SP[2:0]);
    end
  endtask

  task pop_ras;
    output [14:0] a;
    begin
      a = RAS[SP];
      SP = (SP - 1) & 7;
      PSU = (PSU & ~PSU_SP) | (SP[2:0]);
    end
  endtask

  function signed [31:0] fetch_rel_off;
    input [7:0] b;
    reg [6:0] off7;
    begin
      off7 = b[6:0];
      fetch_rel_off = off7[6] ? {25'h1FFFFFF, off7} : {25'h0, off7};
    end
  endfunction

  function [14:0] resolve_addr;
    input [14:0] base;
    input integer ind;
    begin
      if (!ind) resolve_addr = base;
      else resolve_addr = {mrd(base)[6:0], mrd((base + 1) & 15'h7FFF)};
    end
  endfunction

  task alu_add;
    input [7:0] a, b;
    input integer wc;
    output [7:0] r;
    integer carry_in;
    integer s, lo;
    begin
      carry_in = (wc && (PSL & PSL_C)) ? 1 : 0;
      s = a + b + carry_in;
      lo = (a & 8'h0F) + (b & 8'h0F) + carry_in;
      if (s > 255) PSL = PSL | PSL_C; else PSL = PSL & ~PSL_C;
      if (lo > 15) PSL = PSL | PSL_IDC; else PSL = PSL & ~PSL_IDC;
      r = s[7:0];
      if (((~a[7]) && (~b[7]) && r[7]) || (a[7] && b[7] && (~r[7]))) PSL = PSL | PSL_OVF;
      else PSL = PSL & ~PSL_OVF;
    end
  endtask

  task alu_sub;
    input [7:0] a, b;
    input integer wb;
    output [7:0] r;
    integer borrow_in;
    integer d, lo;
    begin
      borrow_in = (wb && !(PSL & PSL_C)) ? 1 : 0;
      d = a - b - borrow_in;
      lo = (a & 8'h0F) - (b & 8'h0F) - borrow_in;
      if (d >= 0) PSL = PSL | PSL_C; else PSL = PSL & ~PSL_C;
      if (lo >= 0) PSL = PSL | PSL_IDC; else PSL = PSL & ~PSL_IDC;
      r = d[7:0];
      if ((a[7] && ~b[7] && ~r[7]) || (~a[7] && b[7] && r[7])) PSL = PSL | PSL_OVF;
      else PSL = PSL & ~PSL_OVF;
    end
  endtask

  task pb_ret;
    reg [14:0] ra;
    begin
      pop_ras(ra);
      IAR = ra;
    end
  endtask

  task execute;
    reg [14:0] op_pc, t, target, eff;
    reg [7:0] op, ob, b1, b2, m, r, old, operand, a8, b8, res;
    integer rn, grp, mode, ind, idx, off, wc, com;
    begin
      op_pc = IAR;

      if (use_pipbug) begin
        if (op_pc == 15'h02B4) begin $write("%c", R[ri(0)]); pb_ret(); disable execute; end
        if (op_pc == 15'h0286) begin R[ri(0)] = $fgetc('h80000000); set_cc(R[ri(0)]); pb_ret(); disable execute; end
        if (op_pc == 15'h008A) begin $write("\r\n"); pb_ret(); disable execute; end
      end

      fetch(op);
      rn = op[1:0];

      if (trace) $display("[%04h] %02h  R0=%02h R1=%02h R2=%02h R3=%02h PSL=%02h SP=%0d",
                          op_pc, op, R[ri(0)], R[ri(1)], R[ri(2)], R[ri(3)], PSL, SP);

      if (op == 8'h40) begin running = 0; disable execute; end
      if (op == 8'hC0) disable execute;
      if (op == 8'h12) begin R[ri(0)] = PSU; set_cc(R[ri(0)]); disable execute; end
      if (op == 8'h13) begin R[ri(0)] = PSL; set_cc(R[ri(0)]); disable execute; end
      if (op == 8'h92) begin PSU = (PSU & PSU_S) | (R[ri(0)] & (PSU_F | PSU_II | PSU_SP)); disable execute; end
      if (op == 8'h93) begin PSL = R[ri(0)]; disable execute; end

      if ((op >= 8'h14) && (op <= 8'h17)) begin if (test_cc(op[1:0])) pop_ras(IAR); disable execute; end
      if ((op >= 8'h34) && (op <= 8'h37)) begin if (test_cc(op[1:0])) begin pop_ras(IAR); PSU = PSU & ~PSU_II; end disable execute; end

      if ((op >= 8'h74) && (op <= 8'h77)) begin
        fetch(m);
        case (op)
          8'h74: PSU = PSU & ~(m & (PSU_F | PSU_II | PSU_SP));
          8'h75: PSL = PSL & ~m;
          8'h76: PSU = PSU |  (m & (PSU_F | PSU_II | PSU_SP));
          8'h77: PSL = PSL | m;
        endcase
        disable execute;
      end

      if (op == 8'hB4) begin fetch(m); PSL = (PSL & ~PSL_CC) | (((PSU & m) == m) ? CC_ZERO : CC_NEG); disable execute; end
      if (op == 8'hB5) begin fetch(m); PSL = (PSL & ~PSL_CC) | (((PSL & m) == m) ? CC_ZERO : CC_NEG); disable execute; end

      if ((op >= 8'h94) && (op <= 8'h97)) begin
        r = R[ri(rn)];
        if ((r[3:0] > 4'd9) || (PSL & PSL_IDC)) r = r + 8'h06;
        if ((r[7:4] > 4'd9) || (PSL & PSL_C)) r = r + 8'h60;
        R[ri(rn)] = r;
        set_cc(r);
        disable execute;
      end

      if ((op >= 8'hF4) && (op <= 8'hF7)) begin
        fetch(m);
        PSL = (PSL & ~PSL_CC) | (((R[ri(rn)] & m) == m) ? CC_ZERO : CC_NEG);
        disable execute;
      end

      if ((op >= 8'h50) && (op <= 8'h53)) begin
        r = R[ri(rn)]; old = r;
        if (PSL & PSL_WC) begin
          r = {((PSL & PSL_C) ? 1'b1 : 1'b0), r[7:1]};
          if (old[0]) PSL = PSL | PSL_C; else PSL = PSL & ~PSL_C;
          if (old[7] ^ r[7]) PSL = PSL | PSL_OVF; else PSL = PSL & ~PSL_OVF;
          if (r[5]) PSL = PSL | PSL_IDC; else PSL = PSL & ~PSL_IDC;
        end else begin
          r = {r[0], r[7:1]};
        end
        R[ri(rn)] = r; set_cc(r); disable execute;
      end

      if ((op >= 8'hD0) && (op <= 8'hD3)) begin
        r = R[ri(rn)]; old = r;
        if (PSL & PSL_WC) begin
          r = {r[6:0], ((PSL & PSL_C) ? 1'b1 : 1'b0)};
          if (old[7]) PSL = PSL | PSL_C; else PSL = PSL & ~PSL_C;
          if (old[7] ^ r[7]) PSL = PSL | PSL_OVF; else PSL = PSL & ~PSL_OVF;
          if (r[5]) PSL = PSL | PSL_IDC; else PSL = PSL & ~PSL_IDC;
        end else begin
          r = {r[6:0], r[7]};
        end
        R[ri(rn)] = r; set_cc(r); disable execute;
      end

      if (op == 8'h9B) begin
        fetch(ob); ind = ob[7]; off = fetch_rel_off(ob);
        t = (IAR & 15'h6000) | (off[12:0]);
        IAR = resolve_addr(t, ind);
        disable execute;
      end

      if (op == 8'hBB) begin
        fetch(ob); ind = ob[7]; off = fetch_rel_off(ob);
        target = (IAR & 15'h6000) | (off[12:0]);
        target = resolve_addr(target, ind);
        push_ras(IAR); IAR = target;
        disable execute;
      end

      if (((op >= 8'h30) && (op <= 8'h33)) || ((op >= 8'h70) && (op <= 8'h73)) || ((op >= 8'h54) && (op <= 8'h57))) begin
        R[ri(rn)] = $fgetc('h80000000);
        set_cc(R[ri(rn)]);
        disable execute;
      end
      if (((op >= 8'hB0) && (op <= 8'hB3)) || ((op >= 8'hF0) && (op <= 8'hF3)) || ((op >= 8'hD4) && (op <= 8'hD7))) begin
        $write("%c", R[ri(rn)]);
        set_cc(R[ri(rn)]);
        disable execute;
      end

      if ((op >= 8'h18) && (op <= 8'h1B)) begin fetch(ob); ind = ob[7]; off = fetch_rel_off(ob); t = (IAR + off) & 15'h7FFF; t = resolve_addr(t, ind); if (test_cc(op[1:0])) IAR = t; disable execute; end
      if ((op >= 8'h1C) && (op <= 8'h1F)) begin fetch(b1); fetch(b2); ind = b1[7]; t = {b1[6:0], b2}; t = resolve_addr(t, ind); if (test_cc(op[1:0])) IAR = t; disable execute; end
      if ((op >= 8'h98) && (op <= 8'h9A)) begin fetch(ob); ind = ob[7]; off = fetch_rel_off(ob); t = (IAR + off) & 15'h7FFF; t = resolve_addr(t, ind); if (!test_cc(op[1:0])) IAR = t; disable execute; end
      if ((op >= 8'h9C) && (op <= 8'h9E)) begin fetch(b1); fetch(b2); ind = b1[7]; t = {b1[6:0], b2}; t = resolve_addr(t, ind); if (!test_cc(op[1:0])) IAR = t; disable execute; end

      grp = -1; mode = (op >> 2) & 2'b11;
      if (op <= 8'h0F) grp = 0;
      else if ((op >= 8'h20) && (op <= 8'h2F)) grp = 1;
      else if ((op >= 8'h40) && (op <= 8'h4F)) grp = 2;
      else if ((op >= 8'h60) && (op <= 8'h6F)) grp = 3;
      else if ((op >= 8'h80) && (op <= 8'h8F)) grp = 4;
      else if ((op >= 8'hA0) && (op <= 8'hAF)) grp = 5;
      else if ((op >= 8'hC0) && (op <= 8'hCF) && (op != 8'hC0) && !((op >= 8'hC4) && (op <= 8'hC7))) grp = 6;
      else if ((op >= 8'hE0) && (op <= 8'hEF)) grp = 7;

      if (grp >= 0) begin
        operand = 8'h00; eff = 15'h0000; ind = 0; idx = 0;

        if (grp == 6) begin
          case (mode)
            0: begin R[ri(rn)] = R[ri(0)]; disable execute; end
            1: begin fetch(ob); disable execute; end
            2: begin fetch(ob); ind = ob[7]; off = fetch_rel_off(ob); eff = (IAR + off) & 15'h7FFF; eff = resolve_addr(eff, ind); mwr(eff, R[ri(rn)]); disable execute; end
            3: begin
              fetch(b1); fetch(b2); ind = b1[7]; idx = b1[6:5]; eff = {b1[4:0], b2}; eff = resolve_addr(eff, ind);
              if (idx == 1) begin R[ri(rn)] = R[ri(rn)] + 1; eff = (eff + R[ri(rn)]) & 15'h7FFF; end
              else if (idx == 2) begin R[ri(rn)] = R[ri(rn)] - 1; eff = (eff + R[ri(rn)]) & 15'h7FFF; end
              else if (idx == 3) eff = (eff + R[ri(rn)]) & 15'h7FFF;
              mwr(eff, R[ri(rn)]);
              disable execute;
            end
          endcase
        end

        case (mode)
          0: operand = R[ri(rn)];
          1: fetch(operand);
          2: begin fetch(ob); ind = ob[7]; off = fetch_rel_off(ob); eff = (IAR + off) & 15'h7FFF; eff = resolve_addr(eff, ind); operand = mrd(eff); end
          3: begin
            fetch(b1); fetch(b2); ind = b1[7]; idx = b1[6:5]; eff = {b1[4:0], b2}; eff = resolve_addr(eff, ind);
            if (idx == 1) begin R[ri(rn)] = R[ri(rn)] + 1; eff = (eff + R[ri(rn)]) & 15'h7FFF; end
            else if (idx == 2) begin R[ri(rn)] = R[ri(rn)] - 1; eff = (eff + R[ri(rn)]) & 15'h7FFF; end
            else if (idx == 3) eff = (eff + R[ri(rn)]) & 15'h7FFF;
            operand = mrd(eff);
          end
        endcase

        a8 = (mode == 0) ? R[ri(0)] : R[ri(rn)];
        b8 = (mode == 0) ? R[ri(rn)] : operand;
        wc = (PSL & PSL_WC) ? 1 : 0;

        case (grp)
          0: begin res = b8; if ((mode == 0) || ((mode == 3) && (idx != 0))) R[ri(0)] = res; else R[ri(rn)] = res; set_cc(res); end
          1: begin res = R[ri(0)] ^ b8; R[ri(0)] = res; set_cc(res); end
          2: begin res = a8 & b8; if ((mode == 0) || ((mode == 3) && (idx != 0))) R[ri(0)] = res; else R[ri(rn)] = res; set_cc(res); end
          3: begin res = a8 | b8; if ((mode == 0) || ((mode == 3) && (idx != 0))) R[ri(0)] = res; else R[ri(rn)] = res; set_cc(res); end
          4: begin alu_add(a8, b8, wc, res); if ((mode == 0) || ((mode == 3) && (idx != 0))) R[ri(0)] = res; else R[ri(rn)] = res; set_cc_add(res); end
          5: begin alu_sub(a8, b8, wc, res); if ((mode == 0) || ((mode == 3) && (idx != 0))) R[ri(0)] = res; else R[ri(rn)] = res; set_cc_sub(res); end
          7: begin
            com = (PSL & PSL_COM) ? 1 : 0;
            if (com) begin
              if (a8 > b8) PSL = (PSL & ~PSL_CC) | CC_POS;
              else if (a8 == b8) PSL = (PSL & ~PSL_CC) | CC_ZERO;
              else PSL = (PSL & ~PSL_CC) | CC_NEG;
            end else begin
              if ($signed(a8) > $signed(b8)) PSL = (PSL & ~PSL_CC) | CC_POS;
              else if ($signed(a8) == $signed(b8)) PSL = (PSL & ~PSL_CC) | CC_ZERO;
              else PSL = (PSL & ~PSL_CC) | CC_NEG;
            end
          end
        endcase
        disable execute;
      end

      $display("WARN [%04h]: unhandled opcode $%02h", op_pc, op);
      run_fault = 1;
      running = 0;
    end
  endtask

  initial begin
    $display("sim2650 v%s (Verilog port)", SIM_VER);

    ROM_START_A = DEF_ROM_START;
    ROM_END_A   = DEF_ROM_END;
    RAM_START_A = DEF_RAM_START;
    RAM_END_A   = DEF_RAM_END;

    trace = 0;
    running = 1;
    breakpt = -1;
    icount = 0;
    maxinstr = 5000000;
    mem_warn_count = 0;
    run_fault = 0;
    use_pipbug = 0;
    entry_point = 15'h0000;

    if ($test$plusargs("trace")) trace = 1;
    if ($test$plusargs("pipbug")) begin
      use_pipbug = 1;
      ROM_START_A = PB_ROM_START;
      ROM_END_A   = PB_ROM_END;
      RAM_START_A = PB_RAM_START;
      RAM_END_A   = PB_RAM_END;
      entry_point = 15'h0440;
      $display("PIPBUG 1 mode: ROM $%04h-$%04h RAM $%04h-$%04h", PB_ROM_START, PB_ROM_END, PB_RAM_START, PB_RAM_END);
    end

    if ($value$plusargs("entry=%h", entry_point)) begin end
    if ($value$plusargs("break=%h", breakpt)) begin end
    if ($value$plusargs("maxinstr=%d", maxinstr)) begin end

    for (i = 0; i < MEM_SIZE; i = i + 1) mem[i] = 8'hFF;
    for (i = 0; i < 8; i = i + 1) begin R[i] = 8'h00; RAS[i] = 15'h0000; end

    if (!$value$plusargs("hex=%s", hexfile)) begin
      $display("ERROR: missing +hex=<memory_file> plusarg");
      $finish(1);
    end

    $readmemh(hexfile, mem);

    IAR = entry_point;
    PSU = PSU_II;
    PSL = 8'h00;
    SP  = 0;

    $display("Running from $%04h...", entry_point);

    while (running && (icount < maxinstr)) begin
      if ((breakpt >= 0) && (IAR == breakpt[14:0])) begin
        $display("*** BREAKPOINT $%04h ***", IAR);
        disable execute;
      end
      execute();
      icount = icount + 1;
    end

    if (icount >= maxinstr) $display("*** Instruction limit (%0d) ***", maxinstr);
    $display("Halted after %0d instructions", icount);
    $display("R0=$%02h R1=$%02h R2=$%02h R3=$%02h", R[ri(0)], R[ri(1)], R[ri(2)], R[ri(3)]);
    $display("IAR=$%04h PSU=$%02h PSL=$%02h CC=%0d", IAR, PSU, PSL, (PSL & PSL_CC) >> 6);

    if (run_fault) $finish(2);
    else if (icount >= maxinstr) $finish(3);
    else $finish(0);
  end

endmodule
