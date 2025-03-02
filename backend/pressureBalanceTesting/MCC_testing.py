from MCC_DAQ import MCC

theMCCstuff = MCC(device_count_118=0)
theMCCstuff.set_hats_ready()
theMCCstuff.set_hats_scan_stop()
theMCCstuff.set_hats_scan_start()
goodstuff = theMCCstuff.get_data()
print(goodstuff)
theMCCstuff.set_hats_scan_stop()
