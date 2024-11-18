onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -label clock -radix binary /testbench/clock
add wave -noupdate -label resetn -radix binary /testbench/reset
add wave -noupdate -label in -radix binary /testbench/in
add wave -noupdate -label cX -radix binary /testbench/cX
add wave -noupdate -label cY -radix binary /testbench/cY
add wave -noupdate -label state -radix binary /testbench/state
add wave -noupdate -divider part1
add wave -noupdate -label vX -radix binary /testbench/vX
add wave -noupdate -label vY -radix binary /testbench/vY
add wave -noupdate -label nextX -radix decimal /testbench/nextX
add wave -noupdate -label nextY -radix decimal /testbench/nextY
add wave -noupdate -label move -radix binary /testbench/move
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
