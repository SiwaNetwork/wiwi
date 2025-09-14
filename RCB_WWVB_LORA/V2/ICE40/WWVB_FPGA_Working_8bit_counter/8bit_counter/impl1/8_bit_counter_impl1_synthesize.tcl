if {[catch {

# define run engine funtion
source [file join {C:/lscc/radiant/2023.2} scripts tcl flow run_engine.tcl]
# define global variables
global para
set para(gui_mode) 1
set para(prj_dir) "C:/Users/julianstj/Desktop/WorkNotes/Projects/PTP/WWVB_SDR/ICE40/Projects/8bit_counter"
# synthesize IPs
# synthesize VMs
# synthesize top design
file delete -force -- 8_bit_counter_impl1.vm 8_bit_counter_impl1.ldc
run_engine_newmsg synthesis -f "8_bit_counter_impl1_lattice.synproj"
run_postsyn [list -a iCE40UP -p iCE40UP5K -t SG48 -sp High-Performance_1.2V -oc Industrial -top -w -o 8_bit_counter_impl1_syn.udb 8_bit_counter_impl1.vm] [list C:/Users/julianstj/Desktop/WorkNotes/Projects/PTP/WWVB_SDR/ICE40/Projects/8bit_counter/impl1/8_bit_counter_impl1.ldc]

} out]} {
   runtime_log $out
   exit 1
}
