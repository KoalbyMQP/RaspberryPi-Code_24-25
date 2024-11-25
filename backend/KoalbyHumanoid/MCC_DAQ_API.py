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
        selected_hats_128 = []
        selected_hats_118 = []

        # Get descriptors for all of the available HAT devices.
        hats_128 = hat_list(filter_by_id_128=DEVICE_COUNT_128)
        hats_118 = hat_list(filter_by_id_118=DEVICE_COUNT_118)
        
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
            self.hats_128 = select_hat_devices(HatIDs.MCC_128, DEVICE_COUNT_128)
            self.hats_118 = select_hat_devices(HatIDs.MCC_118, DEVICE_COUNT_118)

            # Validate the selected channels, set the modes and ranges.
            for i, hat in enumerate(hats_128):
                validate_channels(self.chans_128[i], hat.info().NUM_AI_CHANNELS[self.input_modes_128[i]])
                hat.a_in_mode_write(self.input_modes_128[i])
                hat.a_in_range_write(self.input_ranges_128[i])
            for i, hat in enumerate(hats_118):
                validate_channels(self.chans_118[i], hat.info().NUM_AI_CHANNELS)
            
            # Set the trigger mode for the master device.
            hats_128[MASTER_128].trigger_mode(self.trigger_mode)

        except:
            return [ {selected_hats_128}, {selected_hats_118}, {False} ]
            
        return [ {selected_hats_128}, {selected_hats_118}, {True} ]

    def get_actual_sample_rate(self):
        #actual_rate_118 = self.hats_118[MASTER_128].a_in_scan_actual_rate(len(chans_118[MASTER_128]),
        #                                                 sample_rate)
        actual_rate_128 = self.hats_128[self.master_128].a_in_scan_actual_rate(len(self.chans_128[MASTER_128]),
                                                         self.sample_rate)
        return actual_rate_128
    
    def get_data(self, samples_to_read = 1, timeout = 5):
        samples_per_chan_read_128 = [0] * self.device_count_128
        samples_per_chan_read_118 = [0] * self.device_count_118
        total_samples_per_chan_128 = [0] * self.device_count_128
        total_samples_per_chan_118 = [0] * self.device_count_118
        is_running_triggerwait = True
        is_triggered = False
        is_running_sample = True
        
        for i, hat in enumerate(self.hats_128):
            chan_mask = chan_list_to_mask(self.hans_128[i])
            hat.a_in_scan_start(chan_mask, self.samples_per_channel, self.sample_rate,
                                self.options_128[i])
        for i, hat in enumerate(self.hats_118):
            chan_mask = chan_list_to_mask(self.chans_118[i])
            hat.a_in_scan_start(chan_mask, self.samples_per_channel, self.sample_rate,
                                self.options_118[i])

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

        if not is_running:
            print('\nMCC HAT get_data told it is not running for some reason. Weird...')
            return None
            
        return [self.chans_128, {-1}, samples_per_chan_read_128, {-1}, total_samples_per_chan_128, {-1}, data_128, {-1}, self.chans_118, {-1}, samples_per_chan_read_118, {-1}, total_samples_per_chan_118, {-1}, data_118]
        
        # Display the data for each HAT device
        #for i, hat in enumerate(self.hats_128):
        #    print('HAT 128 {0}:'.format(i))

            # Print the header row for the data table.
        #    print('  Samples Read    Scan Count', end='')
        #    for chan in chans_128[i]:
        #        print('     Channel', chan, end='')
        #    print('')

            # Display the sample count information.
        #    print('{0:>14}{1:>14}'.format(samples_per_chan_read_128[i],
        #                                  total_samples_per_chan_128[i]), end='')

            # Display the data for all selected channels
        #    for chan_idx in range(len(self.chans_128[i])):
        #        if samples_per_chan_read_128[i] > 0:
        #            sample_idx = ((samples_per_chan_read_128[i] * len(self.chans_128[i]))
        #                          - len(self.chans_128[i]) + chan_idx)
        #            print(' {:>12.5f} V'.format(data_128[i][sample_idx]), end='')
        #    print('\n')
        #for i, hat in enumerate(self.hats_118):
        #    print('HAT 118 {0}:'.format(i))

            # Print the header row for the data table.
        #    print('  Samples Read    Scan Count', end='')
        #    for chan in chans_118[i]:
        #        print('     Channel', chan, end='')
        #    print('')

            # Display the sample count information.
        #    print('{0:>14}{1:>14}'.format(samples_per_chan_read_118[i],
        #                                  total_samples_per_chan_118[i]), end='')

            # Display the data for all selected channels
        #    for chan_idx in range(len(self.chans_118[i])):
        #        if samples_per_chan_read_118[i] > 0:
        #            sample_idx = ((samples_per_chan_read_118[i] * len(self.chans_118[i]))
        #                          - len(self.chans_118[i]) + chan_idx)
        #            print(' {:>12.5f} V'.format(data_118[i][sample_idx]), end='')
        #    print('\n')




# Constants
DEVICE_COUNT_128 = 1
DEVICE_COUNT_118 = 1
MASTER_128 = 0


def main():
    """
    This function is executed automatically when the module is run directly.
    """
    hats_118 = []
    hats_128 = []
    # Define the channel list for each HAT device
    chans_118 = [
        {0, 1, 2, 3, 4, 5, 6, 7}
    ]
    chans_128 = [
        {0, 1, 2, 3, 4, 5, 6, 7}
    ]
    # Define the input modes for each MCC 128 ### FOR 128!! ###
    input_modes_128 = [
        AnalogInputMode.SE
    ]
    # Define the input ranges for each MCC 128 ### FOR 128!! ###
    input_ranges_128 = [
        AnalogInputRange.BIP_10V,
        AnalogInputRange.BIP_10V
    ]
    # Define the options for each HAT device
    options_118 = [
        OptionFlags.EXTCLOCK
    ]
    options_128 = [
        OptionFlags.EXTTRIGGER
    ]
    samples_per_channel = 10000
    sample_rate = 1000.0  # Samples per second
    trigger_mode = TriggerModes.RISING_EDGE

    try:
        # Get an instance of the selected hat device object.
        hats_118 = select_hat_devices(HatIDs.MCC_118, DEVICE_COUNT_118)
        hats_128 = select_hat_devices(HatIDs.MCC_128, DEVICE_COUNT_128)

        # Validate the selected channels, set the modes and ranges.
        for i, hat in enumerate(hats_128):
            validate_channels(chans_128[i], hat.info().NUM_AI_CHANNELS[input_modes_128[i]])
            hat.a_in_mode_write(input_modes_128[i])
            hat.a_in_range_write(input_ranges_128[i])
        for i, hat in enumerate(hats_118):
            validate_channels(chans_118[i], hat.info().NUM_AI_CHANNELS)

        # Set the trigger mode for the master device.
        hats_128[MASTER_128].trigger_mode(trigger_mode)

        # Calculate the actual sample rate.
        actual_rate_118 = hats_118[MASTER_128].a_in_scan_actual_rate(len(chans_118[MASTER_128]),
                                                         sample_rate)
        actual_rate_128 = hats_128[MASTER_128].a_in_scan_actual_rate(len(chans_128[MASTER_128]),
                                                         sample_rate)

        #print('MCC 128 multiple HAT example using external clock and',
        #      'external trigger options')
        #print('    Functions demonstrated:')
        #print('      mcc128.trigger_mode')
        #print('      mcc128.a_in_scan_start')
        #print('      mcc128.a_in_scan_status')
        #print('      mcc128.a_in_scan_read')
        #print('      mcc128.a_in_scan_stop')
        #print('      mcc128.a_in_scan_cleanup')
        #print('      mcc128.a_in_mode_write')
        #print('      mcc128.a_in_range_write')
        #print('    Samples per channel:', samples_per_channel)
        #print('    Requested Sample Rate: {:.3f} Hz'.format(sample_rate))
        #print('    Actual Sample Rate: {:.3f} Hz'.format(actual_rate))
        #print('    Trigger type:', trigger_mode.name)

        #for i, hat in enumerate(hats):
            #print('    HAT {}:'.format(i))
            #print('      Address:', hat.address())
            #print('      Input mode: ', input_mode_to_string(input_modes[i]))
            #print('      Input range: ', input_range_to_string(input_ranges[i]))
            #print('      Channels: ', end='')
            #print(', '.join([str(chan) for chan in chans[i]]))
            #options_str = enum_mask_to_string(OptionFlags, options[i])
            #print('      Options:', options_str)

        #print('\n*NOTE: Connect the CLK terminals together on each MCC 128')
        #print('       HAT device being used. Connect a trigger source')
        #print('       to the TRIG input terminal on HAT 0.')

        #try:
        #    input("\nPress 'Enter' to continue")
        #except (NameError, SyntaxError):
        #    pass

        # Start the scan.
        for i, hat in enumerate(hats_128):
            chan_mask = chan_list_to_mask(chans_128[i])
            hat.a_in_scan_start(chan_mask, samples_per_channel, sample_rate,
                                options_128[i])
        for i, hat in enumerate(hats_118):
            chan_mask = chan_list_to_mask(chans_118[i])
            hat.a_in_scan_start(chan_mask, samples_per_channel, sample_rate,
                                options_118[i])

        #print('\nWaiting for trigger ... Press Ctrl-C to stop scan\n')

        try:
            # Monitor the trigger status on the master device.
            wait_for_trigger(hats_128[MASTER_128])
            # Read and display data for all devices until scan completes
            # or overrun is detected.
            read_data(hats_118, hats_128, chans_118, chans_128)

        except KeyboardInterrupt:
            # Clear the '^C' from the display.
            #print(CURSOR_BACK_2, ERASE_TO_END_OF_LINE, '\nAborted\n')
            pass

    except (HatError, ValueError) as error:
        print('\n', error)

    finally:
        for hat in hats_118:
            hat.a_in_scan_stop()
            hat.a_in_scan_cleanup()
        for hat in hats_128:
            hat.a_in_scan_stop()
            hat.a_in_scan_cleanup()


def wait_for_trigger(hat):
    """
    Monitor the status of the specified HAT device in a loop until the
    triggered status is True or the running status is False.

    Args:
        hat (mcc128): The mcc128 HAT device object on which the status will
            be monitored.

    Returns:
        None

    """
    # Read the status only to determine when the trigger occurs.
    is_running = True
    is_triggered = False
    while is_running and not is_triggered:
        status = hat.a_in_scan_status()
        is_running = status.running
        is_triggered = status.triggered


def read_data(hats_118, hats_128, chans_118, chans_128):
    """
    Reads data from the specified channels on the specified DAQ HAT devices
    and updates the data on the terminal display.  The reads are executed in a
    loop that continues until either the scan completes or an overrun error
    is detected.

    Args:
        hats (list[mcc128]): A list of mcc128 HAT device objects.
        chans (list[int][int]): A 2D list to specify the channel list for each
            mcc128 HAT device.

    Returns:
        None

    """
    samples_to_read = 500
    timeout = 5  # Seconds
    samples_per_chan_read_118 = [0] * DEVICE_COUNT_118
    samples_per_chan_read_128 = [0] * DEVICE_COUNT_128
    total_samples_per_chan_118 = [0] * DEVICE_COUNT_118
    total_samples_per_chan_128 = [0] * DEVICE_COUNT_128
    is_running = True

    # Create blank lines where the data will be displayed
    #for _ in range(DEVICE_COUNT * 4 + 1):
    #    print('')
    # Move the cursor up to the start of the data display.
    #print('\x1b[{0}A'.format(DEVICE_COUNT * 4 + 1), end='')
    #print(CURSOR_SAVE, end='')

    while True:
        data_128 = [None] * DEVICE_COUNT_128
        data_118 = [None] * DEVICE_COUNT_118
        # Read the data from each HAT device.
        for i, hat in enumerate(hats_128):
            read_result = hat.a_in_scan_read(samples_to_read, timeout)
            data_128[i] = read_result.data
            is_running &= read_result.running
            samples_per_chan_read_128[i] = int(len(data_128[i]) / len(chans_128[i]))
            total_samples_per_chan_128[i] += samples_per_chan_read_128[i]

            if read_result.buffer_overrun:
                print('\nError: Buffer overrun')
                break
            if read_result.hardware_overrun:
                print('\nError: Hardware overrun')
                break
        for i, hat in enumerate(hats_118):
            read_result = hat.a_in_scan_read(samples_to_read, timeout)
            data_118[i] = read_result.data
            is_running &= read_result.running
            samples_per_chan_read_118[i] = int(len(data_118[i]) / len(chans_118[i]))
            total_samples_per_chan_118[i] += samples_per_chan_read_118[i]

            if read_result.buffer_overrun:
                print('\nError: Buffer overrun')
                break
            if read_result.hardware_overrun:
                print('\nError: Hardware overrun')
                break

        #print(CURSOR_RESTORE, end='')

        # Display the data for each HAT device
        for i, hat in enumerate(hats_128):
            print('HAT 128 {0}:'.format(i))

            # Print the header row for the data table.
            print('  Samples Read    Scan Count', end='')
            for chan in chans_128[i]:
                print('     Channel', chan, end='')
            print('')

            # Display the sample count information.
            print('{0:>14}{1:>14}'.format(samples_per_chan_read_128[i],
                                          total_samples_per_chan_128[i]), end='')

            # Display the data for all selected channels
            for chan_idx in range(len(chans_128[i])):
                if samples_per_chan_read_128[i] > 0:
                    sample_idx = ((samples_per_chan_read_128[i] * len(chans_128[i]))
                                  - len(chans_128[i]) + chan_idx)
                    print(' {:>12.5f} V'.format(data_128[i][sample_idx]), end='')
            print('\n')
        for i, hat in enumerate(hats_118):
            print('HAT 118 {0}:'.format(i))

            # Print the header row for the data table.
            print('  Samples Read    Scan Count', end='')
            for chan in chans_118[i]:
                print('     Channel', chan, end='')
            print('')

            # Display the sample count information.
            print('{0:>14}{1:>14}'.format(samples_per_chan_read_118[i],
                                          total_samples_per_chan_118[i]), end='')

            # Display the data for all selected channels
            for chan_idx in range(len(chans_118[i])):
                if samples_per_chan_read_118[i] > 0:
                    sample_idx = ((samples_per_chan_read_118[i] * len(chans_118[i]))
                                  - len(chans_118[i]) + chan_idx)
                    print(' {:>12.5f} V'.format(data_118[i][sample_idx]), end='')
            print('\n')

        stdout.flush()

        if not is_running:
            break


def select_hat_devices(filter_by_id, number_of_devices):
    """
    This function performs a query of available DAQ HAT devices and determines
    the addresses of the DAQ HAT devices to be used in the example.  If the
    number of HAT devices present matches the requested number of devices,
    a list of all mcc128 objects is returned in order of address, otherwise the
    user is prompted to select addresses from a list of displayed devices.

    Args:
        filter_by_id (int): If this is :py:const:`HatIDs.ANY` return all DAQ
            HATs found.  Otherwise, return only DAQ HATs with ID matching this
            value.
        number_of_devices (int): The number of devices to be selected.

    Returns:
        list[mcc128]: A list of mcc128 objects for the selected devices
        (Note: The object at index 0 will be used as the master).

    Raises:
        HatError: Not enough HAT devices are present.

    """
    selected_hats = []

    # Get descriptors for all of the available HAT devices.
    hats = hat_list(filter_by_id=filter_by_id)
    number_of_hats = len(hats)

    # Verify at least one HAT device is detected.
    if number_of_hats < number_of_devices:
        error_string = ('Error: This example requires {0} MCC 1x8 HATs - '
                        'found {1}'.format(number_of_devices, number_of_hats))
        raise HatError(0, error_string)
    elif number_of_hats == number_of_devices:
        for i in range(number_of_devices):
            if filter_by_id == 128:
                selected_hats.append(mcc128(hats[i].address))
            if filter_by_id == 118:
                selected_hats.append(mcc118(hats[i].address))
    else:
        # Display available HAT devices for selection.
        for hat in hats:
            print('Address ', hat.address, ': ', hat.product_name, sep='')
        print('')

        for device in range(number_of_devices):
            valid = False
            while not valid:
                input_str = 'Enter address for HAT device {}: '.format(device)
                address = int(input(input_str))

                # Verify the selected address exists.
                if any(hat.address == address for hat in hats):
                    valid = True
                else:
                    print('Invalid address - try again')

                # Verify the address was not previously selected
                if any(hat.address() == address for hat in selected_hats):
                    print('Address already selected - try again')
                    valid = False

                if valid:
                    if filter_by_id == 128:
                        selected_hats.append(mcc128(hats[i].address))
                    if filter_by_id == 118:
                        selected_hats.append(mcc118(hats[i].address))

    return selected_hats


if __name__ == '__main__':
    # This will only be run when the module is called directly.
    main()




# Yes, this is modeled after how Adafruit APIs are written
#class MCC_DAQ_API:
#    def __init__(
#        self,
#        input_ranges_128
#    ) -> None :
#        self.input_ranges_128 = input_ranges_128
#    def read_data(self, boardModel: int, copyOfModel: int) -> (int, int,  int, int,  int, int,  int, int):


#class
