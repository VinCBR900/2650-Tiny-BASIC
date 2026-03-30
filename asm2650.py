#!/usr/bin/env python3
# tabs = 4

# Change to True/False to enable/disable color in error and warning messages
USECOLOR = True

"""
This program is a 2650 assembler, based on the assembler published on
https://binnie.id.au/MicroByte/

It has been extensively modified to provide error checking and warnings, and to support
the sytax used by the DASMx disassembler as well as the standard Signetics format.
The assembler is case-insensitive, but case-preserving.
Support for the 2650B processor has been added (additional instructions LDPL, STPL)
See https://ztpe.nl for documentation.
"""

import sys, os, re
from collections import namedtuple
from enum import Enum
import argparse

# Predefined symbols and their values. Others may be added during initialisation in main()
# The index-strings in the table *must* be upper-case.
PredefinedSymbols = {
	# registers
	'R0': 0,
	'R1': 1,
	'R2': 2,
	'R3': 3,
	# Condition codes
	'EQ': 0, # equal
	'GT': 1, # greater than
	'LT': 2, # less than
	'UN': 3, # unconditional
	'Z': 0,	 # zero
	'P': 1,	 # positive
	'N': 2,	 # negative
	'NE': 2, # Not Equal, for TMI instruction
	# PSW upper
	'SENS': 0x80,
	'FLAG': 0x40,
	'II': 0x20,
	# PSW lower
	'IDC': 0x20,
	'RS': 0x10,
	'WC': 0x08,
	'OVF': 0x04,
	'LCOM': 0x02,
	'CAR': 0x01,
}

class SymType(Enum):
	predefined = 0			# builtin symbols
	userdefined = 1			# symbols used in the input code files

# Information on a symbol:
#  - type: from SymType
#  - name: string, preserving case, as defined in the input code files
#  - value: the resolved value of the symbol, or None when as-of-yet undetermined
#  - where: string, describing where the symbol was defined (usually filename:lineno)
# Note that SymInfo[x].name.upper() == x
SymInfo = namedtuple('SymInfo', ['type', 'name', 'value', 'where'])


# Name tuple of a single expression token: its type and value
ExpressionToken = namedtuple('ExpressionToken', ['type','value'])


class UndefinedClass:
	"""An empty class having only one member, representing an undetermined value."""
	def __str__(self): return "UNDEF"
	def __repr__(self): return "UNDEF"

# The one member of this class, indicating an undetermined value.
UNDEF = UndefinedClass()


# Warnings produce output (unless suppressed) but do not cause an exception.
# Errors do procuce an exception.
#
class WarnType(Enum):
	"Warning types"
	instr = 1,		# warn about unusual instructions
	label = 2,		# warn about label redefinitions
	rel = 3,		# warn when relative addressing could have been used
	base = 4,		# warn when ambiguous base is used

class LabelNotFoundException(Exception):
	""" custom exception on errors with referenced symbols """

class AsmException(Exception):
	""" custom exception on general errors (not missing labels) """

class CondException(Exception):
	""" custom exception on errors with conditional assembly """


class Assembler:
	"""
	This class provides the assembler's processing
	Any errors will be raised as exceptions (as above).
	"""
	def __init__(self,options,reporter):
		# Options are passed as properties of an Options object
		self.opt = options
		# Handler for output
		self.rep = reporter
		# Symbol table. Any predefined and user-defined labels will be added to the table.
		# Index must be upper case string: the name of the symbol.
		# Value must be a SymInfo
		self.SymTable = {}
		self.initPass(1)
		# Populate the Symbol Table with the list of predefined symbols
		for s,v in PredefinedSymbols.items():
			self.SymTable[s] = SymInfo(SymType.predefined, s, v, 'predefined')
		# Patterns for expression tokens that can make up the operand
		# These will be matched case-insensitive; only capitals are used below but in matching
		# lowercase letters will also match.
		NAME =	r'(?P<NAME>[A-Z_][A-Z_0-9]*)'
		HEXLS = r"(?P<HEXLS>H'[^']+')"
		HEX =	r"(?P<HEX>H'[A-F0-9]+')"
		OCTLS = r"(?P<OCTLS>O'[^']+')"
		OCT =	r"(?P<OCT>O'[0-7]+')"
		BINLS = r"(?P<BINLS>B'[^']+')"
		BIN =	r"(?P<BIN>B'[0-1]+')"
		DHEX =	r"(?P<DHEX>\$[A-F0-9]+)"
		# a-quote-something-quote, where something is a repetition of EITHER a non-quote OR quote-quote
		ASCII = r"(?P<ASCII>A'([^']|(''))*')"
		# dquote-something-dquote, where something is a repetion of EITHER a non-dquote OR dquote-dquote
		ASTR =	r"(?P<ASTR>\"([^\"]|(\"\"))*\")"
		DECLS = r"(?P<DECLS>D'[^']+')"
		DEC =	r"(?P<DEC>D'\d+')"
		NUM =	r'(?P<NUM>[\+\-]?\d[A-F0-9]*)'	# base is set by the DFLT pseudo-op
		OPEN =	r'(?P<OPEN>\()'
		CLOSE = r'(?P<CLOSE>\))'
		SHL =	r'(?P<SHL>\<\<)'
		SHR =	r'(?P<SHR>\>\>)'
		TIMES = r'(?P<TIMES>\*)'
		DIV =	r'(?P<DIV>/)'
		MOD =	r'(?P<MOD>%)'
		PLUS =	r'(?P<PLUS>\+)'
		MINUS = r'(?P<MINUS>-)'
		EQ =	r'(?P<EQ>\.EQ\.)'
		NE =	r'(?P<NE>\.NE\.)'
		GT =	r'(?P<GT>\.GT\.)'
		LT =	r'(?P<LT>\.LT\.)'
		GE =	r'(?P<GE>\.GE\.)'
		LE =	r'(?P<LE>\.LE\.)'
		NOT =	r'(?P<NOT>\.NOT\.)'
		LAND =	r'(?P<LAND>&)'
		LOR =	r'(?P<LOR>\|)'
		LXOR =	r'(?P<LXOR>\^)'
		SELF =	r'(?P<SELF>\$(?![A-F0-9]))'
		UPPER = r'(?P<UPPER>\<(?!\<))'
		LOWER = r'(?P<LOWER>\>(?!\>))'
		INDEX = r'(?P<INDEX>,R[0-3](?![A-Z_0-9])(,?[+-])?)'
		SEP =	r'(?P<SEP>,\s*)'
		COMMENT=r'(?P<COMMENT>\s.*)'
		CATCHALL=r'(?P<CATCHALL>.+)'
		# A single pattern to match tokens from a string. The order of the names patterns is relevant: the
		# first matching subpattern will be used. Matching is case-insensitive
		self.operand_pat = re.compile('(?i)'+'|'.join([UPPER, LOWER, HEX, DHEX, HEXLS, DEC, DECLS, OCT, OCTLS, BIN, BINLS,
			ASCII, ASTR, INDEX, NAME, NUM, OPEN, CLOSE, SHL, SHR, TIMES, DIV, MOD, PLUS, MINUS,
			EQ, NE, GT, LT, GE, LE, NOT,
			LAND, LOR, LXOR,
			SELF, SEP, COMMENT, CATCHALL]))
		# define all other fields
		self.ambiguousnum = None
		self.CmdLength = None
		self.Comment = None
		self.DataBytes = None
		self.DFLT = None
		self.guard = None
		self.ignorecode = None
		self.line = None
		self.Listing = None
		self.nextPC = None
		self.PC = None
		self.PCuncertain = None
		self.redefines = None
		self.memlimitexception = True

	def initPass(self, Pass):
		# Pass is either 1 or 2. Pass 1 is a syntax check and collects all symbols. It can
		# be executed multiple times, to resolved forward references.
		# Pass 2 creates the code and listing, and does further checks on semantics.
		self.Pass = Pass
		self.CmdLength = 0				# Number of code bytes generated by the current (pseudo)instruction.
		# The current value of the program counter
		self.PC = 0
		# Sometimes the length of an instruction cannot be determined. Thereafter
		# the value of the PC is uncertain.
		self.PCuncertain = False
		# The next value of the program counter. Normally selfPC + size of instruction, but
		# jumps are possible with the ORG pseudo.
		self.nextPC = 0
		self.DFLT = 16 if self.opt.defaultHex else 10 # default base for numerical constants
		self.codesectionslength = [0]	# Number of output bytes in each code section
		self.ignorecode = False			# possibly set to true during IF..ELSE..ENDIF
		self.initLine('')
		self.iflevel = 0				# nesting level of if..endif
		self.guard = 0					# base nesting level of if..endif in INCLUDE files
		self.ifinfo = []				# consists of tuples (condition,else-seen,where-string)
		self.redefines = 0				# number of times a symbol redefinition occurred this pass
		self.memlimitexception = True	# raise exception when PC exceeds 7fff
		# Erase all information on locations
		for s, sym in self.SymTable.items():
			if sym.type==SymType.predefined: continue
			sym = sym._replace(where='')
			self.SymTable[s] = sym

	def initLine(self,line):
		if self.ignorecode: self.CmdLength = 0
		if self.PCuncertain:
			self.PC = 0
		else:
			self.PC = self.nextPC + self.CmdLength
			if self.PC>=0x8000 and self.memlimitexception:
				# Raise this error only once, otherwise it will be raised for each and every
				# instruction after the first instruction that hits 0x8000
				self.memlimitexception = False
				raise AsmException("Instruction exceeds top memory limit")
			self.wdebug(2,None,"PC becomes «0x%04X» (nextPC «0x%04X» + CmdLength «%d»)" % (self.PC,self.nextPC,self.CmdLength))
		self.nextPC = self.PC # Program Counter after instruction
		self.ambiguousnum = ""
		self.CmdLength = 0	  # instruction length in bytes (can be any length, e.g. with DATA pseudo)
		self.Comment = ""	  # Resolved comment
		self.DataBytes = []	  # Result os raw bytes
		self.Listing = ""	  # Result as text line listing
		self.line = line	  # current line from input file

	# Signetics 2650 assembler Pseudo-ops.
	# Some of these are accepted but ignored: END, EJE, PRT, SPC, TITL, PCH, PAG, START
	# Additions:
	# - both DATA and DB can be used to specify memory contents.
	# - both ACON and DW can be used to specify address constants.
	# - DFLT can be used to set the default numbering style ('1' or '16' for hexadecimal, '0' or '10' for decimal)
	# If no DFLT is specified decimal numbering is assumed for compatibility, although
	# this can be changed with the command line option --hex.
	Pseudo =  ('ORG', 'EQU', 'RES', 'END', 'ACON', 'DW', 'DATA', 'DFLT', 'DB',
			   'EJE', 'PRT', 'SPC', 'TITL', 'PAG', 'PCH', 'START',
			   'IF','ELSE','ENDIF', 'WARN', 'ERROR')

	# 2650 Instruction Mnemonic Codes
	# MNEC is the table of opcode names and byte value. Function addrModeAndLength
	# returns the addressing mode and size for an opcode.
	MNEC = {
		# Load/Store
		'LODZ': 0x00, 'LODI': 0x00+4, 'LODR': 0x00+8, 'LODA': 0x00+12,
		'STRZ': 0xC0, 'STRI': 0xC0+4, 'STRR': 0xC0+8, 'STRA': 0xC0+12,
		# Arithmetic
		'ADDZ': 0x80, 'ADDI': 0x80+4, 'ADDR': 0x80+8, 'ADDA': 0x80+12,
		'SUBZ': 0xA0, 'SUBI': 0xA0+4, 'SUBR': 0xA0+8, 'SUBA': 0xA0+12,
		# Logical
		'ANDZ': 0x40, 'ANDI': 0x40+4, 'ANDR': 0x40+8, 'ANDA': 0x40+12,
		'IORZ': 0x60, 'IORI': 0x60+4, 'IORR': 0x60+8, 'IORA': 0x60+12,
		'EORZ': 0x20, 'EORI': 0x20+4, 'EORR': 0x20+8, 'EORA': 0x20+12,
		# Comparison
		'COMZ': 0xE0, 'COMI': 0xE0+4, 'COMR': 0xE0+8, 'COMA': 0xE0+12,
		# Rotate
		'RRR': 0x50,
		'RRL': 0xD0,
		# Branch
		'BCTR': 0x18, 'BCTA': 0x18+4,
		'BCFR': 0x98, 'BCFA': 0x98+4,
		'BRNR': 0x58, 'BRNA': 0x58+4,
		'BIRR': 0xD8, 'BIRA': 0xD8+4,
		'BDRR': 0xF8, 'BDRA': 0xF8+4,
		'BXA': 0x9F,
		'ZBRR': 0x9B,
		# Subroutine/Return
		'BSTR': 0x38, 'BSTA': 0x38+4,
		'BSFR': 0xB8, 'BSFA': 0xB8+4,
		'BSNR': 0x78, 'BSNA': 0x78+4,
		'BSXA': 0xBF,
		'RETC': 0x14, 'RETE': 0x34,
		'ZBSR': 0xBB,
		# Program/Status
		'LPSU': 0x92, 'LPSL': 0x92+1,
		'SPSU': 0x12, 'SPSL': 0x12+1,
		'CPSU': 0x74, 'CPSL': 0x74+1,
		'PPSU': 0x76, 'PPSL': 0x76+1,
		'TPSU': 0xB4, 'TPSL': 0xB4+1,
		# Input/Output
		'REDC': 0x30, 'REDD': 0x70, 'REDE': 0x54,
		'WRTC': 0xB0, 'WRTD': 0xF0, 'WRTE': 0xD4,
		# Misc
		'HALT': 0x40,
		'DAR': 0x94,
		'TMI': 0xF4,
		'NOP': 0xC0,
		# 2650B only
		'LDPL': 0x10,
		'STPL': 0x11
	}

	@classmethod
	def addrModeAndLength(cls,opcd):
		"""
		opcd *must* be a valid opcode!
		Determine Addressing mode (below) and instruction length (1, 2, 3).
		 mode	S	len reg/cc	opnd	indirct indexed X-page	example
		  Z		Z	1	Y		N		N		N		N		LODZ,r1
		  ZM	E	1	N		N		N		N		N		HALT
		  I		I	2	Y		Y		N		N		N		LODI,r0 5
		  IM	EI	2	N		Y		N		N		N		CPSL RS
		  R		R	2	Y		Y		Y/N		N		N		STRR,r0 $+5 / BCTR,UN $+5
		  R0	ER	2	N		Y		Y/N		N		N		ZBRR 5
		  A		A	3	Y		Y		Y/N		Y/N		N		LODA,r0 ADDR,R1,+
		  AB	B	3	Y		Y		Y/N		N		Y		BCTA,GT ADDR
		  A3	EB	3	N		Y		Y/N		Y(R3)	Y		BXA ADDR,R3
		  AC	C	3	N		Y		Y/N		N		Y		LDPL ADDR (2650B only)
		We use this many addressing modes to assist in error checking.
		The S-column corresponds to the Format specification in the Signetics CPU manual.
		"""
		if len(opcd)==3: opcd += " "
		# special branches
		if opcd[:2]=='ZB':	# Zero Branch
			return 'R0', 2
		if opcd in ('BXA ', 'BSXA'):
			return 'A3', 3
		# Structured opcodes
		if opcd[3]=='Z':	# Immediate
			return 'Z', 1
		if opcd[3]=='I':	# Immediate
			return 'I', 2
		if opcd[3]=='R':	# Relative
			return 'R', 2
		if opcd[0]=='B':	# Branch (absolute)
			return 'AB', 3
		if opcd[3]=='A':	# Absolute
			return 'A', 3
		# Other opcodes
		if opcd in ('TMI ', 'WRTE', 'REDE'):
			return 'I', 2
		if opcd[:3]=='CPS' or opcd[:3]=='PPS' or opcd[:3]=='TPS':
			return 'IM', 2
		if opcd[:3]=='LPS' or opcd[:3]=='SPS':
			return 'ZM', 1
		if opcd in ('LDPL', 'STPL'):
			return 'AC', 3
		if opcd in ('HALT', 'NOP '):
			return 'ZM', 1
		# the rest
		return 'Z', 1

	def reschar(self,addr):
		"""
		Returns the byte value of the filler byte to use for RES blocks, and between
		ORG sections. This is a fixed value with all of the --res command line options,
		except for "--res #", which fills a 'random' value. The random value must be
		predictable, so that assembler runs always produce identical output. The value
		is therefore based on the address (the lower byte of the address).
		"""
		# A randomized range(0,256)
		pseudorandom = [196,116,82,246,228,88,137,208,15,121,222,237,162,83,207,23,157,172,229,
			178,130,38,57,128,238,205,18,253,70,235,31,148,160,231,67,50,164,168,113,212,
			145,43,233,170,141,10,7,149,232,97,226,36,140,221,56,150,75,13,11,80,49,68,42,
			211,236,26,171,89,242,114,61,190,65,123,63,85,103,240,92,195,167,255,27,209,
			184,197,46,37,73,106,101,175,166,159,117,74,224,185,183,12,58,179,213,143,
			144,32,191,250,244,147,66,254,55,1,142,22,44,77,99,3,200,127,53,188,180,220,5,
			234,9,109,59,155,84,95,151,17,136,161,182,131,34,25,40,102,28,45,72,189,245,
			125,152,181,199,112,124,210,202,19,135,60,98,169,133,203,198,218,100,41,119,
			174,129,21,153,156,120,8,249,93,247,6,163,176,165,104,78,90,4,243,241,0,154,
			193,215,248,29,139,33,217,107,225,115,111,214,51,158,146,48,94,105,54,81,16,
			206,86,177,96,122,173,227,192,194,230,62,239,71,219,91,69,79,52,30,108,24,
			35,76,126,216,251,134,187,118,47,204,2,110,64,186,39,132,252,138,223,20,14,201,87]

		if type(self.opt.defaultRes) is int:
			# -1 for random, any other value as the filler to use
			if self.opt.defaultRes>=0:
				return self.opt.defaultRes
			else:
				return pseudorandom[ addr % 256 ]
		else:
			# defaultRes is a not an integer but a character
			return ord(self.opt.defaultRes)

	def wdebug(self,level,onpass,dstr):
		"Report debugging information, at a given debug level and (optionally) a certain pass"
		if level and self.opt.debug<level: return
		if onpass and self.Pass!=onpass: return
		# Indent the string based on the debug level
		self.rep.stdoutBlue("%s%s" % ("  " * level, dstr))

	def warnline(self,warntype,wstring):
		"""
		Issue a warning unless
		 - disabled by options
		 - in code that is ignored
		 - suppressed by the literal NOWARN in the comment
		"""
		if warntype==WarnType.instr and not self.opt.instrwarnings:  return
		elif warntype==WarnType.label and not self.opt.labelwarnings:  return
		elif warntype==WarnType.rel and not self.opt.relwarnings:  return
		elif warntype==WarnType.base and not self.opt.basewarnings:  return
		if self.ignorecode:  return
		if (self.Comment and re.search(r'NOWARN',self.Comment)):  return
		# Not ignore, so produce the warning
		self.rep.pwarn(wstring)

	def evaluateToken(self, tok):
		"""
		Evaluate a token
		Returns an integer value (NAME, NUM, HEX, DEC, SELF), a list of integer values
		(HEXLS, DECLS, ASCII, ASTR), a string (COMMENT), None (indicating an undefined
		symbol), or UNDEF (a defined symbol with an undetermined value).
		This ASCII and ASTR strings can be one character long, in which case they are
		a valid operand for Immediate instructions, such as LODI,R0 "@" .
		May raise LabelNotFoundException or AsmException
		"""
		if tok.type=='NAME':
			try:
				# return the value component of the SymInfo
				return self.SymTable[tok.value.upper()].value
			except:
				# raise exception unless we are still in Pass 1, or if the ignorecode flag has been set
				if self.Pass==2 and not self.ignorecode: raise LabelNotFoundException("Undefined symbol " + tok.value)
				return UNDEF
		if tok.type=='NUM':
			try:
				# the token can be ambiguous if it consists of two or more decimal digits, after
				# discarding a leading + and leading zeroes
				num = re.sub(r'^\+?', '', tok.value)
				if re.match(r'^0*[1-9]\d+$',num):
					if len(self.ambiguousnum)>0:
						self.ambiguousnum += ", "
					self.ambiguousnum += num
				return int(tok.value, self.DFLT)		# convert to Decimal/Hex
			except:
				raise AsmException("Invalid number for base-" + str(self.DFLT))
		if tok.type=='HEXLS':
			# String is H'...', strip first two and last character, then split on comma
			# First try each member, only then return the list
			for n in tok.value[2:-1].split(','):
				try:
					int(n,16)
					# Prepend a zero, to catch expressions like '0x1a' --> '00x1a'
					int('0'+n,16)
				except:
					raise AsmException("Invalid H'...' list member \"" + str(n) + "\"")
			return [ int(n,16) for n in tok.value[2:-1].split(',') ]
		if tok.type=='HEX':
			return int(tok.value[2:-1], 16)
		if tok.type=='DHEX':
			return int(tok.value[1:], 16)
		if tok.type=='OCTLS':
			# String is O'...', strip first two and last character, then split on comma
			# First try each member, only then return the list
			for n in tok.value[2:-1].split(','):
				try:
					int(n,8)
					int('0'+n,8)
				except:
					raise AsmException("Invalid O'...' list member \"" + str(n) + "\"")
			return [ int(n,8) for n in tok.value[2:-1].split(',') ]
		if tok.type=='OCT':
			return int(tok.value[2:-1], 8)
		if tok.type=='BINLS':
			# String is B'...', strip first two and last character, then split on comma
			# First try each member, only then return the list
			for n in tok.value[2:-1].split(','):
				try:
					int(n,2)
					int('0'+n,2)
				except:
					raise AsmException("Invalid B'...' list member \"" + str(n) + "\"")
			return [ int(n,2) for n in tok.value[2:-1].split(',') ]
		if tok.type=='BIN':
			return int(tok.value[2:-1], 2)
		if tok.type=='DECLS':
			# String is D'...', strip first two and last character, then split on comma
			# First try each member, only then return the list
			for n in tok.value[2:-1].split(','):
				try:
					int(n,10)
					int('0'+n,10)
				except:
					raise AsmException("Invalid D'...' list member \"" + str(n) + "\"")
			return [ int(n,10) for n in tok.value[2:-1].split(',') ]
		if tok.type=='DEC':
			return int(tok.value[2:-1], 10)
		if tok.type=='ASCII':
			s = tok.value[2:-1]
			s = re.sub("''", "'", s)	# flatten two quotes
			return list(map(ord, s))
		if tok.type=='ASTR':
			def convert_code_to_char(match_obj):
				if match_obj.group() is not None:  return chr( int(match_obj.group(1), 16) )
			s = tok.value[1:-1]
			s = re.sub(r'\\x([0-9a-f][0-9a-f])', convert_code_to_char, s, flags=re.IGNORECASE)
			s = re.sub('""', '"', s)	# flatten two doublequotes
			return [ ord(char) for char in s ]
		if tok.type=='SELF':
			# The program counter, unless its value cannot be determined
			return UNDEF if self.PCuncertain else self.PC
		if tok.type=='COMMENT':
			return tok.value
		if tok.type=='CATCHALL':
			raise AsmException("Invalid expression '%s'" % str(tok.value))

	def eval(self,tokens,prio,sublevel):
		"""
		Evaluate a list of tokens, where 'prio' is the priority of the current
		operation. This means that evaluation should stop when an operator
		is encountered with a priority <= than 'prio'.
		Priority/precedence:
		-1 = ( )		parentheses
		 0 =  <, >		high byte, low byte
		 1 =  ,			list concatenation
		 2 =  ^ |		logical XOR, logical OR
		 3 =  &			logical AND
		 4 = .NOT.		logical negation
		 5 = comparison .EQ. .NE. .GT. .LT. .GE. .LE.
		 6 =  + -		addition, subtraction
		 7 =  << >>		shift left, shift right
		 8 =  * / %		multiplication, division, modulo
		Use parentheses (...) to group expressions
		Returns a tuple:
		- value: the evaluated value of the expression
		- indexreg: value of the index register (or None)
		- status of autoincrement (or None)
		- bracket sublevel (same as sublevel calling parameter, except after closing bracket)
		- list of tokens that have not been evaluated
		"""
		value = None
		indexreg = None
		autoinc = None

		self.wdebug(2,2,str(tokens))
		while len(tokens)>0:
			tok = tokens[0]
			self.wdebug(2,2,str(tok))
			prevtokens = tokens
			tokens = tokens[1:]
			if tok.type=='COMMENT':
				## strip leading whitespace
				comment = re.sub(r'^\s+', '', tok.value)
				self.Comment = comment
				continue
			if tok.type=='OPEN':
				(value, indexreg, autoinc, sublevel, tokens) = self.eval(tokens,-1,sublevel+1)
				continue
			if tok.type=='CLOSE':
				sublevel -= 1
				break
			if tok.type=='NOT':
				(value, indexreg, autoinc, sublevel, tokens) = self.eval(tokens,0,sublevel)
				if value==UNDEF: continue
				if value is None or type(value) is not int:  raise AsmException("Invalid expression")
				val = (255 if value==0 else 0)
				self.wdebug(3,2,"«%s» «%s» = «%s»" % (tok.type,value,val))
				value = val
				continue
			if tok.type in ('UPPER', 'LOWER'):
				(value, indexreg, autoinc, sublevel, tokens) = self.eval(tokens,0,sublevel)
				if value==UNDEF: continue
				if value is None or type(value) is not int:  raise AsmException("Invalid expression")
				if value<0: value += 256
				if tok.type=='UPPER':  value >>= 8 # return upper byte
				if tok.type=='LOWER':  value &= 0xFF # return lower byte
				continue
			# Binary operators
			if tok.type in ('SHL','SHR','TIMES','DIV','MOD','PLUS','MINUS',
							'EQ','NE','GT','LT','GE','LE','LAND','LOR','LXOR'):
				if tok.type in ('LOR','LXOR'):				newprio = 2
				elif tok.type in ('LAND'):					newprio = 3
				elif tok.type in ('NOT'):					newprio = 4
				elif tok.type in ('EQ','NE','GT','LT','GE','LE'): newprio = 5
				elif tok.type in ('PLUS','MINUS'):			newprio = 6
				elif tok.type in ('SHL','SHR'):				newprio = 7
				elif tok.type in ('TIMES','DIV','MOD'):		newprio = 8
				else:  raise AsmException("Invalid operator")
				if prio>=newprio:  return (value, indexreg, autoinc, sublevel, prevtokens)
				# List demotion when calculating and length==1
				if type(value) is list and len(value)==1:  value = value[0]
				leftv = value
				if leftv!=UNDEF:
					if leftv is None or type(leftv) is not int:  raise AsmException("Invalid expression (missing/invalid left operand to '%s')" % tok.value)
				(rightv, indexreg, autoinc, sublevel, tokens) = self.eval(tokens,newprio,sublevel)
				if type(rightv) is list and len(rightv)==1:	 rightv = rightv[0]
				if rightv!=UNDEF:
					if rightv is None or type(rightv) is not int:	 raise AsmException("Invalid expression (missing/invalid right operand to '%s')" % tok.value)
				if leftv==UNDEF or rightv==UNDEF:
					value = UNDEF
				else:
					if tok.type in ('DIV','MOD') and rightv==0: raise AsmException("Invalid expression (div/mod by zero')")
					if tok.type=='PLUS' :  value = leftv  + rightv
					if tok.type=='MINUS':  value = leftv  - rightv
					if tok.type=='TIMES':  value = leftv  * rightv
					if tok.type=='DIV'	:  value = leftv // rightv
					if tok.type=='MOD'	:  value = leftv  % rightv
					if tok.type=='SHL'	:  value = leftv << rightv
					if tok.type=='SHR'	:  value = leftv >> rightv
					if tok.type=='LAND' :  value = leftv  & rightv
					if tok.type=='LOR'	:  value = leftv  | rightv
					if tok.type=='LXOR' :  value = leftv  ^ rightv
					if tok.type=='EQ'	:  value = (255 if leftv  == rightv else 0)
					if tok.type=='NE'	:  value = (255 if leftv  != rightv else 0)
					if tok.type=='GT'	:  value = (255 if leftv   > rightv else 0)
					if tok.type=='LT'	:  value = (255 if leftv   < rightv else 0)
					if tok.type=='GE'	:  value = (255 if leftv  >= rightv else 0)
					if tok.type=='LE'	:  value = (255 if leftv  <= rightv else 0)
				self.wdebug(3,2,"«%s» «%s» «%s» = «%s»" % (leftv,tok.type,rightv,value))
				continue
			if tok.type=='SEP':
				if prio>1:	return (value, indexreg, autoinc, sublevel, prevtokens)
				if value is None:	 raise AsmException("Invalid expression (missing left operand to ',')")
				# start making a list
				if type(value) is not list:	 value = [value]
				(rightv, indexreg, autoinc, sublevel, tokens) = self.eval(tokens,1,sublevel)
				if rightv is None:  raise AsmException("Invalid expression (missing right operand to ',')")
				if rightv==UNDEF:  value.append(rightv)
				if type(rightv) is int:  value.append(rightv)
				if type(rightv) is list:  value = value+rightv
				continue
			if tok.type=='INDEX':
				# Value is like ",R2" or like ",R1-" or like ",R3,+"
				if indexreg is not None:	raise AsmException("Multiple index registers not allowed")
				if tok.value[-1]=='+':
					regval = tok.value[1:-2] if tok.value[-2]==',' else tok.value[1:-1]
					autoinc = 1
				elif tok.value[-1]=='-':
					regval = tok.value[1:-2] if tok.value[-2]==',' else tok.value[1:-1]
					autoinc = 2
				else:
					regval = tok.value[1:]
					autoinc = 3
				newtok = ExpressionToken('NAME', regval)
				indexreg = self.evaluateToken(newtok)
				if len(tokens)>0 and (len(tokens)!=1 or tokens[0].type!='COMMENT'):
					raise AsmException("Invalid expression after index register")
				continue
			# Must be a value
			v = self.evaluateToken(tok)
			if value is None:
				value = v
			elif value==UNDEF or v==UNDEF:
				value = UNDEF
			elif type(value) is int and type(v) is int:
				# String "5+2" is tokenised as num:"5", num:"+2" and not as
				# num:"5",oper:PLUS,num:"2". Similar for "5-2".
				# We correct this by inserting a PLUS token between two consecutive numbers.
				prevtokens.insert(0, ExpressionToken('PLUS', "+"))
				tokens = prevtokens
				# Since we encountered the number already, it may already have been noticed
				# as potentially ambiguous. Do not report the same number twice.
				self.ambiguousnum = ""
			elif type(value) is list and type(v) is int:
				# Same as above. List demotion when value is a list of length 1
				if len(value)==1:
					prevtokens.insert(0, ExpressionToken('PLUS', "+"))
					tokens = prevtokens
					self.ambiguousnum = ""
				else: value.append(v)
			elif type(value) is list and type(v) is list:
				value = value + v
			else:
				raise AsmException("Invalid expression")
		# end while
		return (value, indexreg, autoinc, sublevel, tokens)

	def evaluateExpression(self, operand):
		"""
		Take a string containing operand tokens (optionally followed by a comment),
		and return a list containing:
		- evaluated operand (list of values)
		- indirect addressing (True/False)
		- index register
		- auto-increment (1 for increment, 2 for decrement, 3 for plain indexed)
		All values (except 'indirect') may be None if not present
		"""
		# Indirection must be the very first character, cannot be specified inside the operand
		indirect = False
		if len(operand)>0 and operand[0]=='*':
			indirect = True
			operand = operand[1:]

		# scan Operand and generate a list of Tokens
		scanner = self.operand_pat.scanner(operand)
		tokens = []
		for m in iter(scanner.match, None):
			tokens.append(ExpressionToken(m.lastgroup, m.group()))

		(value, indexreg, autoinc, sublevel, tokens) = self.eval(tokens,-1,0)
		if sublevel<0:	raise AsmException("Invalid expression (unexpected closing bracket)")
		if sublevel>0:	raise AsmException("Invalid expression (missing closing bracket)")
		self.wdebug(2,1,"value=«%s», indirect=«%s», indexreg=«%s», autoinc=«%s»" % (value,indirect,indexreg,autoinc))
		return (value, indirect, indexreg, autoinc)

	def processinstruction(self, instr, rest):
		"""
		Resolve instr array [Opcode , {Register | Condition Code}]
		instr[0] contains a valid opcode.
		"""
		opcode = instr[0]
		OPCD = Assembler.MNEC[opcode]
		AddrMode, self.CmdLength = Assembler.addrModeAndLength(opcode)  # Determine Address mode/ length
		self.wdebug(2,2,"opcode=«%s», addrmode=«%s», cmdlen=«%s»" % (opcode,AddrMode,self.CmdLength))

		regcc = None
		numregcc = None
		# Process the register/cond.code field in the instruction part
		if len(instr)==2 :
			regcc = instr[1].upper()
			if regcc=='':
				raise AsmException("Missing register/condition")
			elif regcc in self.SymTable: # Resolve register
				numregcc = self.SymTable[regcc].value
			else:
				raise AsmException("Unknown register/condition: " + regcc)
		self.wdebug(2,2,"regcc=«%s», numregcc=«%s»" % (regcc,numregcc))

		# Process the operand part
		# Some instructions do not take any operand. In that case, the entire string 'rest' is a comment
		if AddrMode=='ZM' or (AddrMode=='Z' and len(instr)==2):
			opval = None
			indirect = False
			indexreg = None
			autoinc = None
			comment = rest
			self.Comment = comment
		else:
			try:
				(opval,indirect,indexreg,autoinc) = self.evaluateExpression(rest)
				# Since the length of an instruction is known, it is not an issue if the operand is
				# undetermined during the first pass. However, by the time we run the second pass
				# the operand should resolve to a definite value
				if opval==UNDEF:
					if self.Pass==1 or self.ignorecode:
						opval = 0		# assume placeholder value for now
					else:
						raise AsmException("Cannot determine value of operand")
				if type(opval) is list:
					if len(opval)==0:
						raise AsmException("Unexpected empty list")
					elif len(opval)>1:
						raise AsmException("Too many operands for instruction; list or string not permitted")
					else:
						opval = opval[0]
			except LabelNotFoundException as err:
				# Handle during first pass or with the ignorecode flag is set; re-raise on Pass 2
				if self.Pass==2 and not self.ignorecode: raise
				# Fake the results to evaluateExpression()
				opval = 0
				indirect = False
				indexreg = None
				autoinc=None

		self.wdebug(2,2,"opval=«%s», indirect=«%s», indexreg=«%s», autoinc=«%s», comment=«%s»" % (opval,indirect,indexreg,autoinc,self.Comment))
		# If the ignorecode flag is set, the syntax checking can be skipped. Any errors or
		# warnings would be ignored anyway. The DataBytes and CmdLength will be discarded
		# as well, so we might as well return now.
		if self.ignorecode: return

		# Retrieve the operand as it appeared in the source file, for more helpful error and warning messages.
		if len(self.Comment)==0:
			txtoperand = rest
		elif self.Comment==rest:
			txtoperand = ""
		else:
			txtoperand = rest[:-len(self.Comment)]
		txtoperand = re.sub(r'^\s+', '', txtoperand)
		txtoperand = re.sub(r'\s+$', '', txtoperand)

		if AddrMode=='Z' and len(instr)==1 and numregcc is None and opval is not None and 0<=opval<=3:
			# Alternative notation for Zero-addressing is to specify the target register as the operand
			# change "eorz	r0" into canonical notation "eorz,r0"
			numregcc = opval
			opval = None

		# Error checking
		# Opcodes
		if opcode=="ANDZ" and numregcc==0:
			raise AsmException("ANDZ,R0 is not a valid instruction (consider IORZ,R0)")
		if opcode=="STRZ" and numregcc==0:
			raise AsmException("STRZ,R0 is not a valid instruction (consider IORZ,R0)")

		if opcode=="LODZ" and numregcc==0 and self.Pass==2:
			self.warnline(WarnType.instr, "LODZ,R0 is discouraged (consider IORZ,R0)")
		if opcode=="COMZ" and numregcc==0 and self.Pass==2:
			self.warnline(WarnType.instr, "COMZ,R0 has predictable results",)
		if opcode in ('PPSL', 'PPSU') and opval==0 and self.Pass==2:
			self.warnline(WarnType.instr, "%s has no effect (no bits set)" % opcode)
		if opcode in ('CPSL', 'CPSU') and opval==0 and self.Pass==2:
			self.warnline(WarnType.instr, "%s has no effect (no bits cleared)" % opcode)
		if opcode in ('TPSL', 'TPSU') and opval==0 and self.Pass==2:
			self.warnline(WarnType.instr, "%s has predictable results" % opcode)
		if opcode=="TMI" and opval==0 and self.Pass==2:
			self.warnline(WarnType.instr, "TMI has predictable results")

		if not self.opt.allow2650b:
			if opcode=="PPSU" and opval&0x18!=0 and self.Pass==2:
				self.warnline(WarnType.instr, "Flags UF1 and UF2 cannot be set with 2650A")
			if opcode=="TPSU" and opval&0x18!=0 and self.Pass==2:
				self.warnline(WarnType.instr, "TPSU has predictable results (flags UF1 and UF2 are always 0 with 2650A)")

		# Register or condition code
		if AddrMode in ('ZM', 'IM', 'R0', 'AC') and numregcc is not None:
			raise AsmException("Instruction %s does not take a register or condition code." % opcode)
		if numregcc is not None and (numregcc<0 or numregcc>3):  raise AsmException("Invalid register or condition code " + instr[1])
		if (opcode[0]=='B' and opcode[2]=='F' and numregcc==3):	 raise AsmException("Branch on false cannot be unconditional")
		# Register addressing
		if AddrMode=='Z':
			if numregcc is None:	raise AsmException("Missing register")
			if opval is not None:	 raise AsmException("Register addressing cannot take an operand")
			if indirect:  raise AsmException("Indirect addressing not possible with Register addressing")
			if indexreg is not None:	raise AsmException("Register addressing cannot be indexed")
		# Immediate addressing
		if AddrMode[0]=='I':
			if AddrMode=='I' and numregcc is None:  raise AsmException("Missing register")
			if opval is None:	 raise AsmException("Missing immediate value")
			if	opval>255:	raise AsmException("Immediate value must be a single byte (is d'%d', h'%04x')" % (opval,opval))
			if opval<-128:	raise AsmException("Immediate value must be a single byte (is -d'%d', -h'ff%02x')" % (-opval,-opval))
			if opval<0:	 opval += 256
			if indirect:  raise AsmException("Indirect addressing not possible with Immediate addressing")
			if indexreg is not None:	raise AsmException("Immediate addressing cannot be indexed")
		# Relative addressing
		if AddrMode[0]=='R':
			if AddrMode=='R' and numregcc is None:
				# missing condition code / register
				if opcode[0]=='B' or opcode[1]=='B':
					raise AsmException("Missing condition code")
				else:
					raise AsmException("Missing register")
			if opval is None:	 raise AsmException("Missing relative address")
			if opval==UNDEF:  raise AsmException("Cannot determine value of operand")
			if AddrMode=='R':
				offset = opval-(self.PC+2)
				if offset<-64 or offset>63:  raise AsmException("Relative offset to '%s' is %d, and outside permitted range (-64..63)" % (txtoperand,offset))
			if AddrMode=='R0':
				offset = opval
				if opval>0x1000:  offset = 0x2000 - opval
				if offset<-64 or offset>63:	 raise AsmException("Page 0 offset to '%s' is %d, and outside permitted range (-64..63)" % (txtoperand,offset))
			if indexreg is not None:	raise AsmException("Relative addressing cannot be indexed")
		#  mode 'A'
		if AddrMode=='A':
			if numregcc is None:	raise AsmException("Missing register")
			if opval is None:	 raise AsmException("Missing absolute address")
			if opval==UNDEF:  raise AsmException("Cannot determine value of operand")
			if indexreg is not None and numregcc!=0:	raise AsmException("Indexed addressing must operate on R0")
			if opval&0xe000 != self.PC&0xe000:  raise AsmException("Cannot address across pages (consider using indirection)")
		#  mode 'AB'
		if AddrMode=='AB':
			if numregcc is None: raise AsmException("Missing condition code")
			if opval is None:	 raise AsmException("Missing absolute address")
			if opval==UNDEF:  raise AsmException("Cannot determine value of operand")
			if indexreg is not None:	raise AsmException("Indexing is not possible with absolute branch instructions")
			if (opval<0 or opval>0x7fff):  raise AsmException("Branch address outside 0..h'7fff'")
		#  mode 'A3'
		if AddrMode=='A3':
			if numregcc!=3 and indexreg is None: raise AsmException("Instruction %s must be indexed by R3" % opcode)
			if numregcc==3 and indexreg is None:
				indexreg = 3
				numregcc = None
			if numregcc is not None:	 raise AsmException("%s does not take a register/condition code" % opcode)
			if opval is None:	 raise AsmException("Missing absolute address")
			if opval==UNDEF:  raise AsmException("Cannot determine value of operand")
			if indexreg!=3:	 raise AsmException("Instruction %s must be indexed by register R3" % opcode)
			if autoinc!=3:	raise AsmException("Instruction %s cannot use increment/decrement" % opcode)
		# mode 'AC'
		if AddrMode=='AC':
			if indexreg is not None:	raise AsmException("Indexing is not possible with instruction %s" % opcode)
			if opval is None:	 raise AsmException("Missing absolute address")
			if opval==UNDEF:  raise AsmException("Cannot determine value of operand")
		# Absolute addressing
		if AddrMode[0]=='A':
			if opval is None:	 raise AsmException("Missing absolute address")
			if opval==UNDEF:  raise AsmException("Cannot determine value of operand")

		# Warnings
		if AddrMode[0]=='A' and indexreg is not None and (indexreg<1 or indexreg>3) and self.Pass==2:
			self.warnline(WarnType.instr, "R0 is both target register and index register")
		if (AddrMode=='A' and autoinc is None) or AddrMode=='AB' or AddrMode=='A3':
			# A valid operation using absolute addressing
			warno = opval-(self.PC+2)
			if -64<=warno<=63 and self.Pass==2:
				self.warnline(WarnType.rel, "operation could have used relative addressing to '%s'" % (txtoperand))

		# Error when the instruction itself crosses a page boundary
		if (self.PC&0xe000) != ((self.PC+self.CmdLength-1)&0xe000):
			raise AsmException("Instruction crosses page boundary")

		if AddrMode=='A3':
			indexreg = None			# BXA and BSXA always operate on R3. Opcode has no register field
			numregcc = None
		if numregcc is not None:
			OPCD += numregcc	# Modify the opcode. Note that this can always be done by addition.
		if indexreg is not None:
			OPCD += indexreg	# Safe, since numregcc *must* have been zero.

		if AddrMode=='R':
			opval = offset & 0x7f
		if AddrMode[0]=='A':
			if AddrMode in ('AB', 'A3'):
				opval &= 0x7fff
			else:
				opval &= 0x1fff
		if indexreg is not None:
			opval |= autoinc<<13
		if indirect:
			if AddrMode[0]=='R': opval |= 0x80
			if AddrMode[0]=='A': opval |= 0x8000
		if opval is not None: opval = opval & 0xffff

		if OPCD is None:	 return
		self.DataBytes = [OPCD]
		if self.CmdLength==1:
			pass
		elif self.CmdLength==2: # 2 Byte Code
			self.DataBytes.append(opval&0xFF)
		else:  # 3 Byte Code
			self.DataBytes.append(opval>>8)
			self.DataBytes.append(opval&0xFF)

		return

	# instruction *must* be a valid pseudo-op
	# instruction must already be in upper case
	def processpseudo(self, label, instruction, rest):
		self.wdebug(2,None,"label=«%s», pseudo=«%s», rest=«%s»" % (label,instruction,rest))
		# Ignore END, EJE, PRT, SPC, TITL, PAG, PCH, START
		if instruction in ('END', 'EJE', 'PRT', 'SPC', 'TITL', 'PAG', 'PCH', 'START'):
			if label: self.processlabel(label)
			return

		# Process the operand part
		try:
			(opval,indirect,indexreg,autoinc) = self.evaluateExpression(rest)
		except LabelNotFoundException as err:
			# Handle only during first pass, re-raise on Pass==2
			if self.Pass==2: raise
			# Fake the results to evaluateExpression()
			opval = UNDEF
			indirect = False
			indexreg = None

		self.wdebug(2,2,"opval=«%s», indirect=«%s», indexreg=«%s», autoinc=«%s», comment=«%s»"
												 % (opval,indirect,indexreg,autoinc,self.Comment))
		if indirect or indexreg:
			raise AsmException("Invalid operand for pseudo-op " + instruction)

		if instruction=="IF":
			self.iflevel += 1
			where = self.rep.currentplace()
			# Any non-zero value for opval indicates True, zero indicates False.
			condition = UNDEF if opval==UNDEF else (opval!=0)
			self.ifinfo.append( (condition,False,where) )
			self.ignorecode = not self.conditionchain(self.iflevel)
			if opval==UNDEF and self.Pass==2: raise AsmException("Cannot determine value of operand")
			# If it cannot be determined whether the IF or the ELSE branch is used, than the value
			# if the Program Counter becomes guesswork.
			if opval==UNDEF:  self.PCuncertain = True

		if instruction=="ELSE":
			if self.iflevel==self.guard: raise CondException("ELSE without IF")
			(ifcondition,elseseen,where) = self.ifinfo[self.iflevel-1]
			self.ifinfo[self.iflevel-1] = (ifcondition,True,where)
			if elseseen:  raise CondException("More than one ELSE with IF at %s" % where)
			self.ignorecode = not self.conditionchain(self.iflevel)
			if label is not None: raise AsmException("No label allowed for ELSE")

		if instruction=="ENDIF":
			self.iflevel -= 1
			if self.iflevel<self.guard: raise CondException("ENDIF without IF")
			self.ifinfo.pop()
			self.ignorecode = not self.conditionchain(self.iflevel)
			if label is not None: raise AsmException("No label allowed for ENDIF")

		# Issue warning or error, unless the ignorecode flag has been set
		if instruction=="WARN" and self.Pass==2:
			if type(opval) is not list:  raise AsmException("WARN pseudo requires a string")
			# Map ASCII values in opval to chars, and join into a string
			if not self.ignorecode:  self.rep.pwarn(''.join(map(chr,opval)),self.line)
			return
		if instruction=="ERROR" and self.Pass==2:
			if type(opval) is not list:  raise AsmException("ERROR pseudo requires a string")
			# Map ASCII values in opval to chars, and join into a string
			if not self.ignorecode:  raise AsmException( ''.join(map(chr,opval)) )
			return

		# DB/DATA and DW/ACON are the only pseudo-ops that takes a list of values
		if instruction in ("DATA", "DB", "ACON", "DW"):
			if label: self.processlabel(label)
			if opval is None:  opval = []
			if opval==UNDEF:
				if self.Pass==1:
					opval = [0]
				else:
					raise AsmException("Cannot determine operand to %s" % instruction)
			elif type(opval) is not list: opval = [opval]
			if instruction in ("DATA", "DB"):
				for i in range(len(opval)):
					if opval[i]==UNDEF:
						if self.Pass==2: raise AsmException("Cannot determine value of item %d" % (i+1))
						opval[i] = 0
					if type(opval[i]) is int and (opval[i]<-128 or opval[i]>255) and self.Pass==2:
						raise AsmException("DATA values must be a single byte (is %d)" % opval[i])
					if opval[i]<0: opval[i] += 256
			if instruction in ("ACON", "DW"):
				vals = []
				for i in range(len(opval)):
					if opval[i]==UNDEF:
						if self.Pass==2: raise AsmException("Cannot determine value of item %d" % (i+1))
						opval[i] = 0
					if opval[i]<0 and self.Pass==2:
						raise AsmException("ACON values cannot be negative (is %d)" % opval[i])
					if opval[i]>0xffff and self.Pass==2:
						raise AsmException("ACON values must be a two bytes (is %x)" % opval[i])
					vals.append(opval[i] >> 8)
					vals.append(opval[i]  & 0xff)
				opval = vals
			self.DataBytes = opval
			self.CmdLength = len(self.DataBytes)
			return

		if type(opval) is list:
			if len(opval)==1 and instruction=="EQU":
				opval = opval[0]
			else:
				raise AsmException("Too many operands for instruction; list or string not permitted")

		if instruction=="ORG":
			if opval is None: raise AsmException("Missing address")
			# If the operand cannot be determined, then we do not know the value of the Program Counter,
			# which is an issue only during Pass 2.
			self.PCuncertain = opval==UNDEF
			if opval==UNDEF and self.Pass==2: raise AsmException("Cannot determine value of operand")
			if self.opt.segmpadded:
				# Pad the current segment so that it reaches the start of the new segment.
				totallen = 0
				for i in range(len(self.codesectionslength)): totallen += self.codesectionslength[i]
				if totallen>0:
					if self.PC > opval: raise AsmException("ORG addresses must be in increasing order")
					for i in range(opval-self.PC): self.DataBytes.append( self.reschar(i+self.PC) )
			self.codesectionslength.append(0)
			self.PC = opval
			self.nextPC = opval
			self.PCuncertain = (opval==UNDEF)
			if label: self.processlabel(label)

		if instruction=="EQU":
			if opval is None: raise AsmException("Missing operand")
			if label is None: raise AsmException("Missing label")
			if opval==UNDEF and self.Pass==2: raise AsmException("Cannot determine value of operand")
			if type(opval) is int and opval<-128: raise AsmException("EQU value too small")
			if type(opval) is int and opval>0xffff: raise AsmException("EQU value too large")
			self.processlabel(label,opval)

		if instruction=="RES":
			if opval is None: raise AsmException("Missing operand")
			if opval==0 and self.Pass==2: self.warnline(WarnType.instr, "Reserving zero bytes")
			if opval==UNDEF:
				# If the operand cannot be determined, then the size of the RES is unknown. Therefore
				# also the value of the Program Counter after the RES becomes unknown.
				# This is only an issue during Pass 2.
				self.PCuncertain = True
				if self.Pass==2:  raise AsmException("Cannot determine value of operand")
			if type(opval) is int and opval<0: raise AsmException("RES value is negative")
			self.CmdLength = 0 if opval==UNDEF else opval
			self.DataBytes = []
			# When Single Segment:
			# The RES command only outputs bytes to the binary file when there already is
			# data in this codesection. It adds data after other bytes, but will not output
			# bytes if it is the first instruction in its codesection.
			# When Padded: always produce output
			if type(opval) is int:
				if self.opt.segmpadded or self.codesectionslength[ len(self.codesectionslength)-1 ]>0:
					for i in range(opval): self.DataBytes.append( self.reschar(i+self.PC) )
			if label: self.processlabel(label)

		if instruction=="DFLT":
			# valid values for DFLT are: 0, 1, 10, 16
			#  0 or 10 mean decimal numbers
			#  1 or 16 mean hexadecimal numbers
			if opval is None: raise AsmException("Missing operand")
			if label is not None: raise AsmException("No label allowed for DFLT")
			# Note that 0x10 evaluates to 16, and 0x16 evaluates to 22
			if self.DFLT==16:
				if opval==16: opval = 10
				if opval==22: opval = 16
			if opval==0: opval = 10
			if opval==1: opval = 16
			if opval in (10, 16):
				self.DFLT = opval
			else:
				raise AsmException("Invalid numerical base '%s' for DFLT" % str(opval))

	def conditionchain(self,level):
		"""
		Compute the AND of all IF conditions in this nesting and higher. If we're in a
		section that has condition False, then both branches (the IF and the ELSE) of
		this level will have condition False.
		If we're in a section that has condition True (so that all it's ancestors are
		True as well), then return the condition of the IF but negate that condition
		if we're in its ELSE part.
		Any UNDEFined condition is considered True for both the IF-section and the
		ELSE-section. This means that during Pass 1 both sections are inspected by the
		assembler. Once all symbols are resolved, the condition is either True or False
		and only one section is parsed for the final output.
		"""
		if level==0: return True
		# Result from higher levels
		prevcond = self.conditionchain(level-1)
		# Details of the current level
		(ifcondition,elseseen,where) = self.ifinfo[level-1]
		# UNDEF is True for both the IF- and ELSE-section
		return prevcond and (ifcondition==UNDEF or (ifcondition != elseseen))

	def processlabel(self, label, val=None):
		"""
		Add label at 'val' or at self.PC
		"""
		# don't process labels in sections that are skipped during conditional assembly
		if self.ignorecode: return

		ul = label.upper()
		# None means set at current location. If the PCuncertain flag is set then the current
		# location is not known.
		if val is None:  val = UNDEF if self.PCuncertain else self.PC

		self.wdebug(2,None,"Defining %s as %s" % (label,val))
		if not re.match(r'^[A-Z_][A-Z0-9_]*$',ul):
			raise AsmException("Invalid label '%s'" % label)

		# Look up existing symbol information
		if ul in self.SymTable:
			sym = self.SymTable[ul]
		else:
			sym = SymInfo(SymType.userdefined,label,UNDEF,'')
			self.SymTable[ul] = sym

		if sym.type==SymType.predefined and PredefinedSymbols[ul]!=val and self.Pass==2:
			self.warnline(WarnType.label, "redefining builtin symbol '%s' (predefined as %04x)" % (sym.name,PredefinedSymbols[ul]))
		if sym.type!=SymType.predefined and sym.where!='' and sym.value!=val and self.Pass==2:
			self.warnline(WarnType.label, "redefining symbol '%s' (previously defined as %04x at %s)" % (sym.name,sym.value,sym.where))
		if sym.value!=val:  self.redefines += 1
		sym = sym._replace(value = val)
		sym = sym._replace(where = self.rep.currentplace())
		self.SymTable[ul] = sym

	def outputline(self,label,instruction,operand,comment):
		"""
		Format a line in the output listing.
		It is safe to call this function during the first pass.
		"""

		if self.Pass==1: return
		if label is None: label=""
		if instruction is None: instruction=""
		if operand is None: operand=""
		if comment is None: comment=""
		Uinstr = instruction.upper()
		self.wdebug(1,None,"OUT: label=«%s», instruction=«%s», operand=«%s», comment=«%s»" % (label,instruction,operand,comment))

		# The format used
		#		  |1	  |	 2	  |	   3  |		 4|		  |5	  |	 6	  |	   7
		# 12345678901234567890123456789012345678901234567890123456789012345678901234567890
		# addr					  LABEL:		  comment
		# addr: 00 11 22 3				  opcd,cc operand				  comment
		# addr: 00 11 22 3		  shrtlbl opcd,cc operand				  comment
		# addr: 00 11 22 33 44 55				  DATA	  A'This is a long data def'
		# addr: 00 11 22 33 44 55 LongerLabel	  DATA	  A'This is a long data def'
		#						  ; Full line comment

		# Empty line
		if label=="" and Uinstr=="" and comment=="":
			self.Listing = ""
			return

		# Full-line comment
		if label=="" and Uinstr=="" and comment!="":
			self.Listing = "%24s%s" % ("",comment)
			return

		if len(comment)>0 and comment[0]!=';': comment = "; "+comment

		if len(self.ambiguousnum)>0 and Uinstr!="DFLT":
			self.warnline(WarnType.base,"number %s can be ambiguous without a base prefix" % self.ambiguousnum)

		# Full-line label
		if label!="" and Uinstr=="":
			# Skip display of the Program Counter when the ignorecode flag is set
			if self.ignorecode:
				self.Listing = "{0:5s} {1:18s}{2:8s}{3}".format("","",label+':',comment)
			else:
				self.Listing = "{0:04X}: {1:18s}{2:8s}{3}".format(self.PC,"",label+':',comment)
			return

		# Value of EQU symbol will always be listed using a normal label, never a full-line label
		if Uinstr=="EQU":
			v = self.SymTable[label.upper()].value
			if v<0: v += 256
			self.Listing  = "{0:02X}{1:02X}= {2:18s}{3:15s} {4}".format(v//256,v%256,"",label,instruction)
			numsp = 20-len(label)
			numsp = max(numsp, 1)
			numsp = min(numsp, 5)
			self.Listing += " "*numsp
			self.Listing += "{0:15s} {1}".format(operand,comment)
			return

		# Label for code bytes will appear as a full line label if too long
		if (Uinstr not in Assembler.Pseudo and len(label)> 7) or \
		   (Uinstr     in Assembler.Pseudo and len(label)>15):
			if self.ignorecode:
				self.Listing += "      {0:18s}{1:s}:\n".format("",label)
			else:
				self.Listing += "{0:04X}: {1:18s}{2:s}:\n".format(self.PC,"",label)
			label = ""

		if self.CmdLength>0:
			self.Listing += "{0:04X}: ".format(self.PC)
		elif label!="":
			self.Listing += "{0:04X}:   ".format(self.PC)
		else:	# Padding for other non-coding Pseudo
			self.Listing += "{0:8s}".format('')

		# Padding for RES will be abbreviated if it is more than 6 bytes
		if Uinstr=="RES" and len(self.DataBytes)>6:
			for i in range(4):	self.Listing += "{0:02X} ".format(self.DataBytes[i])
			self.Listing += "{0:02X}".format(self.DataBytes[4])
			self.Listing += "... {0:16}{1:8}{2:8}{3}".format(label,instruction,operand,comment)
			return

		# Padding for ORG will not show in the listing, but is preserved because
		# it must appear in the output file.
		keepbytes = self.DataBytes
		if Uinstr=="ORG": self.DataBytes=[]

#		if len(self.DataBytes)!=self.CmdLength: self.Listing += "  "

		# There can be many DATA bytes to output. We only do 6 on the first line
		# and print remaining data bytes on subsequent lines of 6 bytes max
		n = 0
		if 0<len(self.DataBytes)<=3:
			for d in self.DataBytes:
				self.Listing += "%02X " % d
				n += 1
			self.Listing += " "*3*(6-n)
		elif len(self.DataBytes)>3:
			while n<len(self.DataBytes) and n<6:
				d = self.DataBytes[n]
				self.Listing += "%02X " % d
				n += 1
			if n<6:
				self.Listing += " "*3*(6-n)
		else:	# Pseudo - no OPC
			self.Listing += " "*16

		if Uinstr in Assembler.Pseudo:
			self.Listing += "%-15s %-7s" % (label,instruction)
			self.Listing += " %-15s" % operand
		else:
			self.Listing += "%-7s %-7s" % (label,instruction)
			self.Listing += " %-23s" % operand
		if len(comment)>0:
			self.Listing += " %s" % comment

		# print the remainder of the DATA bytes, if any
		while n<len(self.DataBytes):
			self.Listing += "\n"
			self.Listing += "{0:04X}: ".format(self.PC+n)
			d = self.DataBytes[n]
			self.Listing += "%02X " % d
			n += 1
			while n<len(self.DataBytes) and n%6!=0:
				d = self.DataBytes[n]
				self.Listing += "%02X " % d
				n += 1
			# end the line, repeat from the outer while-loop

		# restore DataBytes
		self.DataBytes = keepbytes

	# Process line of Assembler code
	def parseline(self, line):
		"""
		Attempt to assemble a single line of input, and place the result into
		self.Listing and self.DataBytes (if second pass)
		May raise exceptions LabelNotFoundException or AsmException
		"""
		self.wdebug(1,None,"IN:"+line)
		self.initLine(line)

		# Remove any trailing whitespace
		line = re.sub(r'\s+$', '', line)

		# EMPTY LINE - retain in output literally
		if len(line)==0 :
			self.outputline(None,None,None,self.line)
			return

		# FULL LINE COMMENT - retain in output (first non-whitespace char is * or ;)
		firstchar = re.sub(r'^\s*', '', line)
		if firstchar[0]=='*' or firstchar[0]==';':
			self.outputline(None,None,None,self.line)
			return

		# FULL LINE LABEL
		# if symbol ends with ":" with optionally a comment
		m = re.match(r'^([a-zA-Z_][a-zA-Z_0-9]*):(\s+(.*))?',line)
		if m:
			self.processlabel(m.group(1))
			self.outputline(m.group(1),None,None,m.group(3))
			return

		# Split line of Assembler code into component fields
		#	Label	Instruction [operand]
		#	The operand is optional, and may contain a Comment
		try:
			label, instruction, *rest = re.split('\\s+',line, maxsplit=2)
		except Exception as e:
			# Not enough values to unpack
			raise AsmException("Missing instruction after label '%s'" % line)

		if label=="": label=None
		instr = instruction.upper().split(",")
		rest = rest[0] if len(rest)>0 else ""

		if len(instr)==0:
			raise AsmException("Missing instruction")

		# If the ignorecode flag is set, then still try to parse the line so that it
		# can be properly formatted by outputline.
		if instruction.upper() in Assembler.Pseudo:
			self.processpseudo(label, instruction.upper(), rest)
		elif instr[0] in Assembler.MNEC:
			if label: self.processlabel(label)
			if (instr[0]=='LDPL' or instr[0]=='STPL') and not self.opt.allow2650b:
				if not self.ignorecode:  raise AsmException("Instruction %s is for 2650B only (consider using --allow2650b)" % instr[0])
			else:
				self.processinstruction(instr,rest)
		elif self.ignorecode:
			# If skipping a section in conditional assembly, then retain in output literally
			# when it does not contain a valid instruction or pseudo.
			self.outputline(None,None,None,self.line)
			return
		else:
			raise AsmException("Unknown instruction " + instr[0])

		# If in an IF-ELSE-ENDIF section that should not generate code, then pretend
		# not to have data
		if self.ignorecode:
			self.DataBytes = []
			self.CmdLength = 0

		self.codesectionslength[ len(self.codesectionslength)-1 ] += len(self.DataBytes)

		# Remove the comment from rest to obtain the operand itself
		operand = rest if self.Comment is None or len(self.Comment)==0 else rest[:-(len(self.Comment))]
		operand = re.sub('\\s+$', '', operand)
		self.outputline(label,instruction,operand,self.Comment)
# END class Assembler


class Options:
	"Options for Assembler objects and related functions"
	def __init__(self):
		self.debug = 0
		self.segmpadded = True
		self.segmsingle = False
		self.allow2650b = False
		self.defaultRes = 0
		self.defaultHex = False
		self.instrwarnings = True
		self.labelwarnings = True
		self.relwarnings = True
		self.basewarnings = True
# END class Options


class Reporter:
	"""
	Handles the output of listing, errors and warnings to:
	 - to stdout
	 - to the listfile (must set the listfile handle first)
	"""
	def __init__(self):
		# listfile can have three values:
		#  - None, for no listing
		#  - 0 (integer), for listing to stdout
		#  - File object, for listing to that file (must be open for writing)
		self.listfile = None
		self.rewind()
		# define all other fields
		self.filename = None
		self.lineno = None
		self.line = None
		# ANSI escape codes to colorize the output, when supported
		if (USECOLOR):
			self.BLUE  = "\033[1;34m"
			self.RED	  = "\033[1;31m"
			self.RESET = "\033[0;0m"
		else:
			self.BLUE  = ""
			self.RED	  = ""
			self.RESET = ""

	def rewind(self):
		self.errors = 0
		self.warnings = 0
		self.newfile(None)

	def newfile(self,name):
		self.filename = name
		self.lineno = 0
		self.line = None

	def nextline(self,line):
		self.lineno += 1
		self.line = line

	def currentplace(self):
		return "%s:%d" % (os.path.basename(self.filename),self.lineno)

	def pwarn(self,warnstring,withLine=True):
		# Write to stdout
		where = self.currentplace()+"| " if withLine else ""
		self.stdoutBlue("%sWarning: %s." % (where,warnstring))
		# If the listfile is a true file, then echo the line on stdout
		if self.listfile and self.listfile!=1:
			if withLine and self.line: self.stdoutFileAndLine()
			self.plist("Warning: %s" % warnstring)
		self.warnings += 1

	def perr(self,errorstring,withLine=True):
		# Write to stdout
		# Write to stdout
		where = self.currentplace()+"| " if withLine else ""
		self.stdoutRed("%sError: %s." % (where,errorstring))
		# If the listfile is a true file, then echo the line on stdout
		self.plist("ERROR: %s" % errorstring)
		if withLine and self.line:
			self.stdoutFileAndLine()
			self.plist("ERROR:\t\t\t%s" % self.line)
		self.errors += 1

	def plist(self,msg):
		if self.listfile is None:
			return
		elif self.listfile==1:
			print(msg, file=sys.stdout)
		else:
			print(msg, file=self.listfile)

	def stdoutFileAndLine(self):
		print("%s| %s" % (self.currentplace(),self.line))

	def stdoutBlue(self,s):
		print("%s%s%s" % (self.BLUE, s, self.RESET))

	def stdoutRed(self,s):
		print("%s%s%s" % (self.RED, s, self.RESET))
# END class Reporter


###############################################################
# Utility functions

def AbortException(err,ex):
	print(err)
	print("	 "+str(ex))
	sys.exit(1)


def entab(s, tabsize=8):
	"""
	Return a copy of s in which spaces have been replaced by tabs.
	Parameter s can be a multiline string (a string containing \n).
	The function steps through all characters in the string, from left to right.
	Non-spaces are copied into the output string, but spaces are "saved up".
	Whenever a space is encountered at a tab position, a tab is output instead of
	the previously encountered spaces.
	"""
	out = ""
	i = 0			# column position into string s (offset to left margin)
	n = 0			# number of spaces encountered pending output
	for char in s:
		i += 1
		if char==" ":
			n += 1
			if i%tabsize==0:
				out += "\t" if n>1 else " "
				n = 0
		elif char=="\n":
			# any trailing spaces (when n>0) will be skipped
			out += "\n"
			n = 0
			i = 0
		else:
			out += " " * n
			out += char
			n = 0
	# strip trailing spaces
	return re.sub(r'\s+$','', out)


def readFile(filename):
	"""
	Read an entire 2650 assembler file, and return it as an array of strings.
	If the filename as specified cannot be found, it is retried using each of the
	directories in the Include list.
	"""
	global IncDirs
	found = False
	incdirs = list(IncDirs)
	incdirs.insert(0,'')		# prepend the current directory
	for pref in incdirs:
		fn = os.path.join(pref,filename)
		if os.path.isfile(fn):
			found = True
			break
	if not found:
		AbortException("File '%s' not found or unreadable." % filename, '')

	# 2650 Assembler files by necessity are small; just read whole file into memory
	try:
		infile = open(fn, 'r')
		Code = infile.read()
		# Remove STX, ETX and Ctrl-Z characters. These are used in the
		# encoding of several old-style ASCII text files.
		Code = re.sub(r'\x02', '', Code)
		Code = re.sub(r'\x03', '', Code)
		Code = re.sub(r'\x1A', '', Code)
		infile.close()
	except Exception as e:
		AbortException("File '%s' not found or unreadable: " % filename, e)
	return Code.splitlines()


def PassOne(fnam,opt):
	"""
	Process a file and all files referenced with INCLUDE. This is a pass one,
	in which all symbols are defined and syntax is checked.
	"""
	global IncDirs,a,reporter
	IncDirs.insert(0, os.path.dirname(os.path.abspath(fnam)))
	codelines = readFile(fnam)
	reporter.newfile(fnam)

	for line in codelines:
		reporter.nextline(line)

		m = re.match(r'^INCLUDE(\*?)\s(.+)',line)
		if m:
			# Save guard and lineno
			beforeguard = a.guard
			a.guard = a.iflevel
			currlineno = reporter.lineno
			# Prevent errors when the file does not exist or cannot be found
			if not a.ignorecode:  PassOne(m.group(2),opt)
			# Restore guard, lineno and filename
			a.guard = beforeguard
			reporter.lineno = currlineno
			reporter.filename = fnam
		else:
			try:
				a.parseline(line)
			except LabelNotFoundException as err:
				# Ignore label errors during the first pass
				pass
			except AsmException as err:
				# Ignore other errors during the first pass
				pass
			except CondException as err:
				reporter.perr(err)
			except Exception as e:
				if opt.debug>=1: raise # re-raise the exception
				reporter.perr("Syntax eror, leading to exception (%s)" % e)
	IncDirs = IncDirs[1:]
	# Report any unclosed IFs, and close them before returning to the parent file (or main).
	while a.iflevel>a.guard:
		(conf,elseseen,where) = a.ifinfo[a.iflevel-1]
		reporter.perr("Missing ENDIF for IF at "+where)
		a.iflevel -= 1
		a.ifinfo.pop()


def PassTwo(fnam,opt,codefile,showlisting=True):
	"""
	Process a file and all files referenced with INCLUDE. This is pass two,
	in which actual output is generated.
	"""
	global IncDirs,a,reporter
	IncDirs.insert(0, os.path.dirname(os.path.abspath(fnam)))
	codelines = readFile(fnam)
	reporter.newfile(fnam)

	for line in codelines:
		reporter.nextline(line)

		m = re.match(r'^INCLUDE(\*?)\s(.+)',line)
		if m:
			# Save guard and lineno
			beforeguard = a.guard
			a.guard = a.iflevel
			currlineno = reporter.lineno
			# transform INCLUDE into INCLUDE* when the ignorecode flag is set
			if showlisting:
				reporter.plist("\t\t\tINCLUDE%s %s" % ("*" if a.ignorecode else m.group(1),m.group(2)))
			if m.group(1)=='*':
				# use ignorecode to prevent errors when the file cannot be found
				if not a.ignorecode:  PassTwo(m.group(2),opt,codefile,showlisting=False)
			else:
				# use ignorecode to prevent errors when the file cannot be found
				if not a.ignorecode:  PassTwo(m.group(2),opt,codefile,showlisting)
			# Restore guard, lineno and filename
			a.guard = beforeguard
			reporter.lineno = currlineno
			reporter.filename = fnam
		else:
			# When the ignorecode flag is set try to parse the line as normal, so that a properly
			# formatted output line can be printed. But do not show any Program Counter or data
			# bytes, and ignore warnings and errors. Just copy the line, but formatted when possible.
			try:
				a.parseline(line)
				if showlisting:  reporter.plist(entab(a.Listing))
				# Stop producing output when an error has been detected
				if len(a.DataBytes)>0 and codefile is not None and reporter.errors==0:
					codefile.write(bytes(a.DataBytes))
			except LabelNotFoundException as err:
				if not a.ignorecode:  reporter.perr(err)
			except AsmException as err:
				if not a.ignorecode:  reporter.perr(err)
			except Exception as e:
				if opt.debug>=1: raise # re-raise the exception
				reporter.perr("Syntax eror, leading to exception (%s)" % e)
	IncDirs = IncDirs[1:]


###############################################################
# Global Variables

IncDirs = []			# include directories, when opening files
args = None				# command line argumenrs
a = None				# Assembler object
reporter = Reporter()	# Report errors and warnings to various locations

VersionString = "2.4.1"
VersionMajor = 2
VersionMinor = 4

###############################################################
# Main

def main():
	global a, IncDirs, args
	codefile = None

	# Parse command line for options
	parser = argparse.ArgumentParser(description='Process 2650 Assembler Code -- version '+VersionString,
									 epilog="See https://ztpe.nl/asm2650 for the manual.")
	# Flags set by command line options
	allwarnings = ("base","instr","label","rel","none")
	segoptions = ("padded","single")
	parser.add_argument(dest='infile', help='input source code file')
	parser.add_argument('-B', '--allow2650b', action='store_true', help='enable instructions specific to 2650B-variant')
	parser.add_argument('-d', '--debug', action='count', default=False, help='enable debugging')
	parser.add_argument('-H', '--hex', action='store_true', help='unprefixed numerical constants are hexadecimal')
	parser.add_argument('-I', '--include', dest='Dir', action='append', default=["$ASM2650INC"], help='directory for INCLUDE')
	parser.add_argument('-l', dest='ListOut', action='store', help='output text listing file')
	parser.add_argument('-o', dest='CodeOut', action='store', help='output binary code file')
	parser.add_argument('-r', '--res', action='store', default="0", help='initialise reserved memory')
	parser.add_argument('--segments', dest='Segments', choices=segoptions, help='how to handle multiple code segments (default=padded)')
	parser.add_argument('-Sp', action='store_true', default=True, help='same as --segments padded')
	parser.add_argument('-Ss', action='store_true', help='same as --segments single')
	parser.add_argument('-W', '--nowarn', action='append', default=[], dest='Warnings', choices=allwarnings, help='disable specified warnings')
	args = parser.parse_args()

	# Prepare the Assembler options
	opt = Options()
	opt.defaultHex = args.hex
	opt.allow2650b = args.allow2650b
	opt.debug = args.debug

	# Warnings and options
	opt.basewarnings = ('base' not in args.Warnings)
	opt.relwarnings = ('rel' not in args.Warnings)
	opt.labelwarnings = ('label' not in args.Warnings)
	opt.instrwarnings = ('instr' not in args.Warnings)
	if 'none' in args.Warnings:
		opt.basewarnings = False
		opt.relwarnings = False
		opt.labelwarnings = False
		opt.instrwarnings = False

	# the --res option can take a number (decimal or hex with 0x prefix) , a single character,
	# or the special value # (meaning random-but-always-the-same)
	if re.match(r'^[0-9]+$',args.res):
		opt.defaultRes = int(args.res)
	elif re.match(r'^0x[0-9a-fA-F]+$',args.res):
		opt.defaultRes = int(args.res,0)
	elif args.res=="#":
		opt.defaultRes = -1
	elif re.match(r'^.$',args.res):
		opt.defaultRes = args.res
	else:
		AbortException("Invalid argument to --res","")
	if type(opt.defaultRes) is int and opt.defaultRes>255:
		AbortException("Invalid argument to --res","")

	# Only one of single/padded will be true. Default is single; padded precedes single.
	if args.Segments == 'single': args.Ss = True
	if args.Segments == 'padded': args.Sp = True
	if args.Sp: args.Ss = False
	opt.segmpadded = args.Sp
	opt.segmsingle = args.Ss

	# Check validity of -I arguments
	IncDirs = []
	for path in args.Dir:
		path = os.path.expandvars(path)
		for d in path.split(":"):
			d = os.path.normpath(d)
			d = os.path.expanduser(d)
			d = os.path.expandvars(d)
			if len(d)>0 and os.path.isdir(d): IncDirs.append(d)

	# Output options
	if args.ListOut:
		try:
			if args.ListOut=="-": reporter.listfile = 1		# special value indicating stdout
			else: reporter.listfile = open(args.ListOut, 'w')
		except Exception as e:
			AbortException("Invalid listing file specified:", e)
	if args.CodeOut:
		try:
			codefile = open(args.CodeOut , 'wb')
		except Exception as e:
			AbortException("No (or invalid) binary file specified:", e)
	# If no output options are specified, dump the listing to stdout
	if reporter.listfile is None and codefile is None: reporter.listfile = 1	# special value indicating stdout

	# Add other predefined symbols that are not fixed
	if args.allow2650b:
		PredefinedSymbols["UF1"] = 0x10
		PredefinedSymbols["UF2"] = 0x08
		PredefinedSymbols["USE2650B"] = 0xff			# equals True
	else:
		PredefinedSymbols["USE2650B"] = 0x00			# equals False
	PredefinedSymbols["VERSIONMAJOR"] = VersionMajor
	PredefinedSymbols["VERSIONMINOR"] = VersionMinor

	# Create the one assembler object.
	a = Assembler(options=opt, reporter=reporter)

	# Do pass 1, until all symbols are resolved, or at most 10 times
	# If any errors are encountered, then terminate immediately.
	# Not all forward references can be resolved in one run. Sometimes a second cycle
	# is necessary, or in rare cases a third. A limit of 10 is practically limitless,
	# unless the symbol cannot be resolved at all.
	# Stop trying when the maximum number of runs is reached, or the last re-run did not
	# make any changes.
	numruns = 0
	undefsymbols = True
	a.redefines = 999
	while undefsymbols and numruns<10 and (numruns==1 or a.redefines>0):
		numruns += 1
		if args.debug>=1: reporter.stdoutRed("***FIRST PASS, run %d **" % numruns)
		reporter.rewind()
		a.initPass(1)
		PassOne(args.infile,opt)

		# Terminate on errors
		if (reporter.errors>0):
			# Attempt to remove previous list and codefiles, as they are definitely not valid anymore
			try:
				if args.ListOut and os.path.exists(args.ListOut): os.remove(args.ListOut)
				if args.CodeOut and os.path.exists(args.CodeOut): os.remove(args.CodeOut)
			except Exception:
				pass
			AbortException("%d errors and %d warnings encountered. Stopping." % (reporter.errors,reporter.warnings),"")
			# no return from AbortException

		# See if a.SymTable contains any undetermined symbol values. If so, another run is
		# necessary to try and resolve those symbols.
		undefsymbols = any( map( (lambda s: s.value==UNDEF), a.SymTable.values()) )
	# end while-loop Pass 1

	# do pass 2 - print listing and binary outputs
	# If any symbols remain unresolved, then proper error messages will be given in Pass 2.
	if args.debug>=1: reporter.stdoutRed("***SECOND PASS**")
	reporter.rewind()
	if len(a.codesectionslength)==1:
		reporter.pwarn("No origin specified. Assuming origin 0; this is likely incorrect",False)

	a.initPass(2)
	PassTwo(args.infile,opt,codefile)

	nzcs = 0	# number of codesections with non-zero length
	for x in a.codesectionslength:
		if x>0: nzcs += 1
	if (nzcs>1 and args.Ss):
		reporter.perr("Multiple output sections (using ORG pseudo-ops) not allowed; use --segments padded",False)

	# The symbol table is sorted alphabetically, and contains all user-defined symbols
	# plus all predefined symbols that have been modified.
	reporter.plist('\n; Symbol table')
	reporter.plist(';-------------')
	for x in sorted(a.SymTable, key=str.lower):
		sym = a.SymTable[x]
		# The SymTable may contain definitions of symbols that were not encountered
		# during pass 2 because of resolved IF-conditions. Skip those.
		if sym.where=='': continue
		# User-defined plus modified Predefined symbols
		if sym.type==SymType.userdefined or sym.value!=PredefinedSymbols[x]:
			if sym.value==UNDEF:
				reporter.plist("UNDEF %s" % (sym.name))
			else:
				v = sym.value
				if v<0: v += 256
				reporter.plist("%04X  %s" % (v,sym.name))
	reporter.plist("\n%d errors and %d warnings encountered." % (reporter.errors,reporter.warnings))

	if (reporter.errors>0):
		try:
			if args.CodeOut and os.path.exists(args.CodeOut): os.remove(args.CodeOut)
		except Exception:
			pass

	# Show errors, unless the listing was sent to the console
	if reporter.errors+reporter.warnings>0 and reporter.listfile is None:
		print("%d errors and %d warnings encountered." % (reporter.errors,reporter.warnings))

	if codefile is not None: codefile.close()
	if type(reporter.listfile) is not int and reporter.listfile is not None:
		reporter.listfile.close()

if __name__=='__main__':
	main()
	sys.exit(reporter.errors)
