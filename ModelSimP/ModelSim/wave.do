onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -label clock -radix binary /testbench/clock
add wave -noupdate -label resetn -radix binary /testbench/resetn
add wave -noupdate -label in -radix hexadecimal /testbench/in
add wave -noupdate -label inEnable -radix binary /testbench/inEnable
add wave -noupdate -label state -radix binary /testbench/state
add wave -noupdate -label length -radix decimal /testbench/length
add wave -noupdate -divider part1
add wave -noupdate -label paddleX -radix decimal /testbench/paddleX
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {10000 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 80
configure wave -valuecolwidth 40
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {0 ps} {120 ns}
