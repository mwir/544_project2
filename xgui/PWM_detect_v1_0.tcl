# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "RESET_POLARITY_LOW" -parent ${Page_0}
  ipgui::add_param $IPINST -name "SIMULATE" -parent ${Page_0}
  ipgui::add_param $IPINST -name "SIMULATE_FREQUENCY_CNT" -parent ${Page_0}


}

proc update_PARAM_VALUE.RESET_POLARITY_LOW { PARAM_VALUE.RESET_POLARITY_LOW } {
	# Procedure called to update RESET_POLARITY_LOW when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.RESET_POLARITY_LOW { PARAM_VALUE.RESET_POLARITY_LOW } {
	# Procedure called to validate RESET_POLARITY_LOW
	return true
}

proc update_PARAM_VALUE.SIMULATE { PARAM_VALUE.SIMULATE } {
	# Procedure called to update SIMULATE when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.SIMULATE { PARAM_VALUE.SIMULATE } {
	# Procedure called to validate SIMULATE
	return true
}

proc update_PARAM_VALUE.SIMULATE_FREQUENCY_CNT { PARAM_VALUE.SIMULATE_FREQUENCY_CNT } {
	# Procedure called to update SIMULATE_FREQUENCY_CNT when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.SIMULATE_FREQUENCY_CNT { PARAM_VALUE.SIMULATE_FREQUENCY_CNT } {
	# Procedure called to validate SIMULATE_FREQUENCY_CNT
	return true
}


proc update_MODELPARAM_VALUE.RESET_POLARITY_LOW { MODELPARAM_VALUE.RESET_POLARITY_LOW PARAM_VALUE.RESET_POLARITY_LOW } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.RESET_POLARITY_LOW}] ${MODELPARAM_VALUE.RESET_POLARITY_LOW}
}

proc update_MODELPARAM_VALUE.SIMULATE { MODELPARAM_VALUE.SIMULATE PARAM_VALUE.SIMULATE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.SIMULATE}] ${MODELPARAM_VALUE.SIMULATE}
}

proc update_MODELPARAM_VALUE.SIMULATE_FREQUENCY_CNT { MODELPARAM_VALUE.SIMULATE_FREQUENCY_CNT PARAM_VALUE.SIMULATE_FREQUENCY_CNT } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.SIMULATE_FREQUENCY_CNT}] ${MODELPARAM_VALUE.SIMULATE_FREQUENCY_CNT}
}

