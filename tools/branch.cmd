@grep -P -n "(?m)^\s*BCTR,(?!UN\b)\S+\s+(\S+)\s*\r?\n\s*BCT[RA],UN\s+\S+\s*\r?\n\s*\1\s*:" %1
