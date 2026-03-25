# dump_banks.gdb
# This script automatically dumps memory for both Bank A and Bank B.
#
# To use this script, start GDB and then load it with:
# (gdb) source Tools/GDB_and_OpenOCD/dump_banks.gdb
#
# Then analyze the dumps with your favorite Binary Analyzer:
# - For example you can use Bless on Linux or even some VSCode Hex Editors for just viewing the binary or performing simple operations.
# - For reverse-engineering or some in-depth analysis you could even use things like Ghirda, but since you have source code that's not really necessary lol 

echo Dumping Bank A memory range (0x10040000 to 0x10220000)...\n
dump binary memory bankA_dump.bin 0x10040000 0x10220000
echo Bank A dump complete: bankA_dump.bin\n

echo Dumping Bank B memory range (0x10220000 to 0x10400000)...\n
dump binary memory bankB_dump.bin 0x10220000 0x10400000
echo Bank B dump complete: bankB_dump.bin\n
