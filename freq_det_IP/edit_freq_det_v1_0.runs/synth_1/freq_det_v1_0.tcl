# 
# Synthesis run script generated by Vivado
# 

set_param gui.test TreeTableDev
debug::add_scope template.lib 1
set_msg_config -id {HDL 9-1061} -limit 100000
set_msg_config -id {HDL 9-1654} -limit 100000

create_project -in_memory -part xc7a100tcsg324-1
set_param project.compositeFile.enableAutoGeneration 0
set_param synth.vivado.isSynthRun true
set_property webtalk.parent_dir c:/users/colten/dropbox/_school/ece544_embedded_programming/project_two/544_project2/freq_det_ip/edit_freq_det_v1_0.cache/wt [current_project]
set_property parent.project_path c:/users/colten/dropbox/_school/ece544_embedded_programming/project_two/544_project2/freq_det_ip/edit_freq_det_v1_0.xpr [current_project]
set_property default_lib xil_defaultlib [current_project]
set_property target_language Verilog [current_project]
set_property ip_repo_paths {
  C:/Users/Colten/Dropbox/_SCHOOL/ECE544_Embedded_Programming/Project_Two/544_project2/freq_det_IP/freq_det_1.0
  C:/Users/Colten/Dropbox/_SCHOOL/ECE544_Embedded_Programming/ip_repo
} [current_project]
read_verilog -library xil_defaultlib {
  c:/users/colten/dropbox/_school/ece544_embedded_programming/project_two/544_project2/freq_det_ip/edit_freq_det_v1_0.srcs/sources_1/imports/544_project2/averaging_freq_det.v
  c:/users/colten/dropbox/_school/ece544_embedded_programming/project_two/544_project2/freq_det_ip/freq_det_1.0/hdl/freq_det_v1_0_S00_AXI.v
  c:/users/colten/dropbox/_school/ece544_embedded_programming/project_two/544_project2/freq_det_ip/freq_det_1.0/hdl/freq_det_v1_0.v
}
catch { write_hwdef -file freq_det_v1_0.hwdef }
synth_design -top freq_det_v1_0 -part xc7a100tcsg324-1
write_checkpoint -noxdef freq_det_v1_0.dcp
catch { report_utilization -file freq_det_v1_0_utilization_synth.rpt -pb freq_det_v1_0_utilization_synth.pb }