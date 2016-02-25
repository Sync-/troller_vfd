target remote :3333

def xc
shell make clean && make

flash
continue
end

def flash
monitor reset halt
file adc.elf
load
end

def mrh
monitor reset halt
end
