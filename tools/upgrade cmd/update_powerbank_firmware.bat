blhost.exe -u -- flash-erase-all
blhost.exe -u -- write-memory 0x8000 PowerBank.bin
blhost.exe -u -- reset
pause