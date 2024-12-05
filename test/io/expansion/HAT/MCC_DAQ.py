# This is just based off of the sample code.
# https://github.com/mccdaq/daqhats?_ga=2.228317470.265389854.1730633306-695559576.1729839657

from __future__ import print_function
from sys import stdout
from daqhats import hat_list, mcc118, mcc128, OptionFlags, HatIDs, TriggerModes, \
    HatError, AnalogInputMode, AnalogInputRange
from daqhats_utils import enum_mask_to_string, chan_list_to_mask, \
    validate_channels, input_mode_to_string, input_range_to_string
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class MCC():
    def __init__(self, device_count_128=1, device_count_118=1, master_128=0, 
                 chans_128 = [ {0, 1, 2, 3, 4, 5, 6, 7} ],
                 chans_118 = [ {0, 1, 2, 3, 4, 5, 6, 7} ],
                 input_modes_128 = [ AnalogInputMode.SE ],
                 input_ranges_128 = [ AnalogInputRange.BIP_10V ],
                 options_128 = [ OptionFlags.EXTTRIGGER ],
                 options_118 = [ OptionFlags.EXTCLOCK ],
                 samples_per_channel = 10000, sample_rate = 1000.0,
                 trigger_mode = TriggerModes.RISING_EDGE
                 ):
        """
        This function initializes the MCC HATs in software.

        Args:
            device_count_128 (int): Number of MCC128 boards. Defaults to 1.
            device_count_118 (int): Number of MCC118 boards. Defaults to 1.
            master_128 (int): Zero index of which MCC128 board provides the master
                clock. Defaults to 0. Assumes MCC128 board controls clock.
            chans_128: List of list of channels for MCC128 boards. Defaults to
                [ {0, 1, 2, 3, 4, 5, 6, 7} ]
                but can be expanded to
                [ {0, 1, 2, 3, 4, 5, 6, 7}, {1, 2, 3, 4, 6} ]
                or any other combination.
            chans_118: List of list of channels for MCC128 boards. Defaults to
                [ {0, 1, 2, 3, 4, 5, 6, 7} ]
                but can be expanded to
                [ {0, 1, 2, 3, 4, 5, 6, 7}, {1, 2, 3, 4, 6} ]
                or any other combination.
            input_modes_128: FOR 128 BOARDS ONLY! List of input modes for MCC128
                boards. Defaults to [ AnalogInputMode.SE ].
            input_ranges_128: FOR 128 BOARDS ONLY! List of input ranges for MCC128
                boards. Defaults to [ AnalogInputRange.BIP_10V ].
            options_128: List of option flags for MCC128 boards. Defaults to
                [ OptionFlags.EXTTRIGGER ].
            options_118: List of option flags for MCC128 boards. Defaults to
                [ OptionFlags.EXTCLOCK ].
            samples_per_channel (int): Number of samples per channel. Defaults to 10000.
            sample_rate: Number of samples per second. Defaults to 1000.0.
            trigger_mode: When on the clock signal to trigger. Defaults to
                TriggerModes.RISING_EDGE. 

        """
        
        # Constants
        self.device_count_128 = device_count_128
        self.device_count_118 = device_count_118
        self.master_128 = master_128
        self.chans_128 = chans_128
        self.chans_118 = chans_118
        self.input_modes_128 = input_modes_128
        self.input_ranges_128 = input_ranges_128
        self.options_128 = options_128
        self.options_118 = options_118
        self.samples_per_channel = samples_per_channel
        self.sample_rate = sample_rate # samples per second
        self.trigger_mode = trigger_mode


        self.hats_128 = []
        self.hats_118 = []

    def set_hats_ready(self):
        """
        This function makes sure all the MCC HATs are recognized and loaded in. \n
        As long as the last value is true, all is well, and the first two lists are the list of 128 HATs and the list of 118 HATs
        respectively. If not, the first two are either the number of HATs found of each type or the list of HATs found of each type.
        It will fail if the number found does not match the number initialized with the class. Use common sense to determine which
        it is.

        Args:
            self: The MCC class.
        
        Returns:
            results: [ { Integer number of 128 HATs found if it failed in counting the HATs or a list of 128 HATs. }, \n
                { Integer number of 118 HATs found if it failed in counting the HATs or a list of 118 HATs. }, \n
                { Did it work? Will be true if it worked, or false if something went wrong. } ]
        """
        
        selected_hats_128 = []
        selected_hats_118 = []

        # Get descriptors for all of the available HAT devices.
        hats_128 = hat_list(filter_by_id_128=self.device_count_128)
        hats_118 = hat_list(filter_by_id_118=self.device_count_118)
        
        number_of_hats_128 = len(hats_128)
        number_of_hats_118 = len(hats_118)

        if (number_of_hats_128 != self.device_count_128 or 
            number_of_hats_118 != self.device_count_118):
            return [ {number_of_hats_128}, {number_of_hats_118}, {False} ]
        for i in range(number_of_hats_128):
            selected_hats_128.append(mcc128(hats_128[i].address))
        for i in range(number_of_hats_118):
            selected_hats_118.append(mcc118(hats_118[i].address))
        
        try:
            # Get an instance of the selected hat device object.
            self.hats_128 = select_hat_devices(HatIDs.MCC_128, self.device_count_128)
            self.hats_118 = select_hat_devices(HatIDs.MCC_118, self.device_count_118)

            # Validate the selected channels, set the modes and ranges.
            for i, hat in enumerate(hats_128):
                validate_channels(self.chans_128[i], hat.info().NUM_AI_CHANNELS[self.input_modes_128[i]])
                hat.a_in_mode_write(self.input_modes_128[i])
                hat.a_in_range_write(self.input_ranges_128[i])
            for i, hat in enumerate(hats_118):
                validate_channels(self.chans_118[i], hat.info().NUM_AI_CHANNELS)
            
            # Set the trigger mode for the master device.
            hats_128[self.master_128].trigger_mode(self.trigger_mode)

        except:
            return [ {selected_hats_128}, {selected_hats_118}, {False} ]
            
        return [ {selected_hats_128}, {selected_hats_118}, {True} ]

    def set_hats_scan_stop(self):
        """
        This function stops the data scan. This is called by get_data.

        Args:
            self: The MCC class.
        
        Returns:
            results (Boolean): If the scan was stopped.
        """

        for hat in self.hats_118:
            hat.a_in_scan_stop()
            hat.a_in_scan_cleanup()
        for hat in self.hats_128:
            hat.a_in_scan_stop()
            hat.a_in_scan_cleanup()
        return True
    
    def set_hats_scan_start(self):
        """
        This function starts the data scan. This is called by get_data.

        Args:
            self: The MCC class.
        
        Returns:
            results (Boolean): If the scan was started.
        """

        for i, hat in enumerate(self.hats_128):
            chan_mask = chan_list_to_mask(self.hans_128[i])
            hat.a_in_scan_start(chan_mask, self.samples_per_channel, self.sample_rate,
                                self.options_128[i])
        for i, hat in enumerate(self.hats_118):
            chan_mask = chan_list_to_mask(self.chans_118[i])
            hat.a_in_scan_start(chan_mask, self.samples_per_channel, self.sample_rate,
                                self.options_118[i])
        return True

    def get_actual_sample_rate(self):
        """
        This function gets the sample rate based on the master HAT.

        Args:
            self: The MCC class.
        
        Returns:
            actual_rate_128: If the scan was stopped.
        """
        actual_rate_128 = self.hats_128[self.master_128].a_in_scan_actual_rate(len(self.chans_128[self.master_128]),
                                                         self.sample_rate)
        
        return actual_rate_128
    
    def get_data(self, samples_to_read = 1, timeout = 5):
        """
        This function gets the data from the HATs. This also calls set_hat_scan_start and set_hats_scan_stop.

        Args:
            self: The MCC class.
            samples_to_read (int): How many samples to read.
            timeout (int): How long (in seconds) to wait if it's running slow.
        
        Returns:
            results: [self.chans_128, {-1}, \n
                samples_per_chan_read_128, {-1}, \n
                total_samples_per_chan_128, {-1}, \n
                data_128, {-1}, \n
                self.chans_118, {-1}, \n
                samples_per_chan_read_118, {-1}, \n
                total_samples_per_chan_118, {-1}, \n
                data_118
        """

        samples_per_chan_read_128 = [0] * self.device_count_128
        samples_per_chan_read_118 = [0] * self.device_count_118
        total_samples_per_chan_128 = [0] * self.device_count_128
        total_samples_per_chan_118 = [0] * self.device_count_118
        is_running_triggerwait = True
        is_triggered = False
        is_running_sample = True
        
        self.set_hats_scan_start()

        # Monitor the trigger status on the master device.
        # Read the status only to determine when the trigger occurs.
        while is_running_triggerwait and not is_triggered:
            status = self.hats_128[self.master_128].a_in_scan_status()
            is_running_triggerwait = status.running
            is_triggered = status.triggered

        # Read and output data for all devices until scan completes
        # or overrun is detected.
        data_128 = [None] * self.device_count_128
        data_118 = [None] * self.device_count_118
        # Read the data from each HAT device.
        for i, hat in enumerate(self.hats_128):
            read_result = hat.a_in_scan_read(samples_to_read, timeout)
            data_128[i] = read_result.data
            is_running &= read_result.running
            samples_per_chan_read_128[i] = int(len(data_128[i]) / len(self.chans_128[i]))
            total_samples_per_chan_128[i] += samples_per_chan_read_128[i]

            if read_result.buffer_overrun:
                print('\nMCC HAT Error: Buffer overrun')
                return None
            if read_result.hardware_overrun:
                print('\nMCC HAT Error: Hardware overrun')
                return None
        for i, hat in enumerate(self.hats_118):
            read_result = hat.a_in_scan_read(samples_to_read, timeout)
            data_118[i] = read_result.data
            is_running &= read_result.running
            samples_per_chan_read_118[i] = int(len(data_118[i]) / len(self.chans_118[i]))
            total_samples_per_chan_118[i] += samples_per_chan_read_118[i]

            if read_result.buffer_overrun:
                print('\nMCC HAT Error: Buffer overrun')
                return None
            if read_result.hardware_overrun:
                print('\nMCC HAT Error: Hardware overrun')
                return None
        
        stdout.flush()

        if not is_running_sample:
            print('\nMCC HAT get_data told it is not running for some reason. Weird...')
            return None
        
        self.set_hats_scan_stop()

        return [self.chans_128, {-1}, samples_per_chan_read_128, {-1}, total_samples_per_chan_128, {-1}, data_128, {-1}, self.chans_118, {-1}, samples_per_chan_read_118, {-1}, total_samples_per_chan_118, {-1}, data_118]

