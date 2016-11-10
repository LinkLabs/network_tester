define gg_reconnect
    target remote localhost:2331
    mon speed auto
    mon endian little
    mon flash download = 1
    mon flash device = EFM32GG232F1024
    mon reset 0
end

gg_reconnect
