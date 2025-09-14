lappend auto_path "C:/lscc/radiant/2023.2/scripts/tcl/simulation"
package require simulation_generation
set ::bali::simulation::Para(DEVICEPM) {ice40tp}
set ::bali::simulation::Para(DEVICEFAMILYNAME) {iCE40UP}
set ::bali::simulation::Para(PROJECT) {simulation}
set ::bali::simulation::Para(MDOFILE) {}
set ::bali::simulation::Para(PROJECTPATH) {C:/Users/julianstj/Desktop/WorkNotes/Projects/PTP/WWVB_SDR/ICE40/Projects/8bit_counter/simulation}
set ::bali::simulation::Para(FILELIST) {"C:/Users/julianstj/Desktop/WorkNotes/Projects/PTP/WWVB_SDR/ICE40/Projects/8bit_counter/source/count_attr.v" "C:/Users/julianstj/Desktop/WorkNotes/Projects/PTP/WWVB_SDR/ICE40/Projects/8bit_counter/source/simple_iq.v" "C:/Users/julianstj/Desktop/WorkNotes/Projects/PTP/WWVB_SDR/ICE40/Projects/8bit_counter/source/led_pattern.v" "C:/Users/julianstj/Desktop/WorkNotes/Projects/PTP/WWVB_SDR/ICE40/Projects/8bit_counter/source/testbench.v" }
set ::bali::simulation::Para(GLBINCLIST) {}
set ::bali::simulation::Para(INCLIST) {"none" "none" "none" "none"}
set ::bali::simulation::Para(WORKLIBLIST) {"work" "work" "work" "work" }
set ::bali::simulation::Para(COMPLIST) {"VERILOG" "VERILOG" "VERILOG" "VERILOG" }
set ::bali::simulation::Para(LANGSTDLIST) {"Verilog 2001" "Verilog 2001" "Verilog 2001" "Verilog 2001" }
set ::bali::simulation::Para(SIMLIBLIST) {pmi_work ovi_ice40up}
set ::bali::simulation::Para(MACROLIST) {}
set ::bali::simulation::Para(SIMULATIONTOPMODULE) {cascaded_cic_filter_tb}
set ::bali::simulation::Para(SIMULATIONINSTANCE) {}
set ::bali::simulation::Para(LANGUAGE) {VERILOG}
set ::bali::simulation::Para(SDFPATH)  {}
set ::bali::simulation::Para(INSTALLATIONPATH) {C:/lscc/radiant/2023.2}
set ::bali::simulation::Para(MEMPATH) {}
set ::bali::simulation::Para(UDOLIST) {}
set ::bali::simulation::Para(ADDTOPLEVELSIGNALSTOWAVEFORM)  {1}
set ::bali::simulation::Para(RUNSIMULATION)  {1}
set ::bali::simulation::Para(SIMULATIONTIME)  {100}
set ::bali::simulation::Para(SIMULATIONTIMEUNIT)  {ns}
set ::bali::simulation::Para(SIMULATION_RESOLUTION)  {default}
set ::bali::simulation::Para(ISRTL)  {1}
set ::bali::simulation::Para(HDLPARAMETERS) {}
::bali::simulation::ModelSim_Run
