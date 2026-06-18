# execute to unbundle tools, 
# Assembler: ./asm2650 source.asm output.hex (source.LST file available)
#            ./asm2650 source.asm -s (dumps symbols to stderr)
# Simulator: ./pipbug_wrap -t program.hex
#            ./pipbug_wrap --help (display usage)

# fix windows CRLF issues
tr -d '\r' < /mnt/user-data/uploads/bundle.txt | base64 -d > bundle.tar.gz
tar xzf bundle.tar.gz -C /home/claude/tools
cd /home/claude/tools && make