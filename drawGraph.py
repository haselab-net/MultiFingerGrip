import struct
import os
import sys
import argparse

try:
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError:
    print("Error: matplotlib and numpy are required. Please install them by running:")
    print("pip install matplotlib numpy")
    sys.exit(1)

def get_latest_log_file(directory='.'):
    """
    Finds the latest log directory and returns the path to a .bin file inside it.
    If multiple .bin files exist, it prompts the user to choose one.
    """
    # Find all directories starting with 'log_' in the given directory
    try:
        log_dirs = [d for d in os.listdir(directory) if os.path.isdir(os.path.join(directory, d)) and d.startswith('log_')]
    except FileNotFoundError:
        return None

    if not log_dirs:
        return None

    # Sort directories by name to find the latest one (e.g., 'log_20231209...')
    latest_dir = sorted(log_dirs, reverse=True)[0]
    latest_dir_path = os.path.join(directory, latest_dir)

    # Find all .bin files in the latest log directory
    try:
        log_files = [f for f in os.listdir(latest_dir_path) if f.endswith('.bin')]
    except FileNotFoundError:
        return None
        
    if not log_files:
        return None

    # If there's only one, just return it
    if len(log_files) == 1:
        selected_file = os.path.join(latest_dir_path, log_files[0])
        print(f"Automatically selected the only log file: {selected_file}")
        return selected_file

    # If there are multiple, sort them and ask the user
    # Sort files numerically based on the part before '.bin'
    try:
        log_files.sort(key=lambda f: int(f.split('.')[0].split('_')[0]))
    except (ValueError, IndexError):
        # Fallback to lexical sort if parsing fails
        log_files.sort()

    print(f"\nMultiple log files found in the latest log directory '{latest_dir_path}':")
    for i, file_name in enumerate(log_files):
        print(f"  [{i}] {file_name}")

    # Prompt user for selection
    while True:
        try:
            # Default to the last (latest) file
            choice_str = input(f"Enter the number of the file to plot (default: {len(log_files) - 1}): ")
            if not choice_str:
                choice = len(log_files) - 1
            else:
                choice = int(choice_str)
            
            if 0 <= choice < len(log_files):
                return os.path.join(latest_dir_path, log_files[choice])
            else:
                print("Invalid number. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")

def custom_plot_function(data_lists, t):
    """
    Define a custom function to plot.
    This function is manually configured by the user.

    Args:
        data_lists (dict): A dictionary containing all data series.
                           Keys are: 't', 'load_pos', 'pointer_pos', 'grip_force',
                                     'lugre_z', 'lugre_dz', 'lugre_v', 'lugre_T',
                                     'is_static_friction', 'friction_force', 'vibration_force', 'mass'.
                           Values are numpy arrays.
        t (np.array): The time vector (already filtered by start/end times).

    Returns:
        A tuple (data, title) where:
        - data (np.array): The data to be plotted. Must have the same length as t.
        - title (str): The title for the subplot.
        
        If you don't want to plot anything, return (None, None).
    """
    # --- Example: Plot the difference between pointer and load position in X ---
    # try:
    #     data = data_lists['pointer_pos'][:, 0] - data_lists['load_pos'][:, 0]
    #     title = 'Pointer Pos X - Load Pos X [m]'
    #     return data, title
    # except (KeyError, IndexError) as e:
    #     print(f"Error in custom_plot_function: {e}")
    #     return None, None

    # --- User can define their custom plot here ---
    # To disable this plot, just return (None, None)
    t_diff = np.copy(t)
    lugre_T_dt = np.insert(np.diff(data_lists['lugre_T']) / 1.0e-3, 0, 0)

    # CLIP (0>x)
    cliped_T_dt = np.clip(-lugre_T_dt, 0, None)
    slip_z = data_lists['lugre_v'][:, 1] - data_lists['lugre_dz'][:, 1]
    a = 0#0.992
    #slip_z = data_lists['is_static_friction']
    # echo decay
    for i in range(1, len(slip_z)):
        slip_z[i] = 1.0*slip_z[i] + a * slip_z[i - 1]

    
    fa1 = 20 # [Hz]
    fa2 = 200 # [Hz]
    A1 = 1.0
    A2 = 0.0
    vibration = A1 * cliped_T_dt * np.sin(2*np.pi*fa1*t) + A2 * np.abs(slip_z) * np.sin(2*np.pi*fa2*t)

    data = vibration

    return data,"vibration"

def read_and_print_condition(f):
    """
    Reads and prints the Condition data from the start of the file.
    Returns a dictionary with the condition parameters and leaves the file
    pointer right after the condition block.
    """
    # sizeof(Condition) is 72 bytes
    # I (4) + 4x (4) + union (48) + d (8) + d (8) = 72
    condition_data_size = 72
    chunk = f.read(condition_data_size)
    if len(chunk) < condition_data_size:
        print("Warning: Could not read full condition data block (file too short).")
        f.seek(0) # Go back to start of file
        return None

    condition = {}
    
    # Unpack friction_model first to decide how to parse the rest
    # In Springhead, COULOMB=0, LUGRE=2. We assume non-Coulomb is LuGre.
    friction_model, = struct.unpack_from('<I', chunk)
    condition['friction_model'] = friction_model

    print("--- Simulation Condition ---")

    if friction_model == 0: # Coulomb model
        # Format: I (4) + 4x (4) + 2d (16) + 32x (32) + d (8) + d (8) = 72 bytes
        format_str = '<I 4x 2d 32x d d'
        unpacked = struct.unpack(format_str, chunk)
        condition['model_type'] = 'Coulomb'
        coulomb_params = { 'mu0': unpacked[1], 'mu':  unpacked[2] }
        condition['coulomb'] = coulomb_params
        condition['mass0'] = unpacked[3]
        condition['dmdt'] = unpacked[4]

        print(f"  Friction Model: Coulomb (ID={friction_model})")
        for key, val in coulomb_params.items():
            print(f"  - {key:<7}: {val}")

    else: # LuGre model (or other)
        # Format: I (4) + 4x (4) + 6d (48) + d (8) + d(8) = 72 bytes
        format_str = '<I 4x 6d d d'
        try:
            unpacked = struct.unpack(format_str, chunk)
            condition['model_type'] = 'LuGre'
            lugre_params = {
                'sigma0': unpacked[1], 'sigma1': unpacked[2], 'sigma2': unpacked[3],
                'A':      unpacked[4], 'B':      unpacked[5], 'C':      unpacked[6],
            }
            condition['lugre'] = lugre_params
            condition['mass0'] = unpacked[7]
            condition['dmdt'] = unpacked[8]
            
            print(f"  Friction Model: LuGre (ID={friction_model})")
            for key, val in lugre_params.items():
                print(f"  - {key:<7}: {val}")
        except struct.error:
            print(f"  Friction Model: Unknown (ID={friction_model})")
            condition['model_type'] = 'Unknown'
            # Try to read mass and dmdt at the end anyway
            try:
                mass0_val, dmdt_val = struct.unpack_from('<2d', chunk, offset=56)
                condition['mass0'] = mass0_val
                condition['dmdt'] = dmdt_val
            except struct.error:
                condition['mass0'] = 'N/A'
                condition['dmdt'] = 'N/A'
    
    print(f"  Initial Mass: {condition.get('mass0', 'N/A')} kg")
    print(f"  Mass Rate (dmdt): {condition.get('dmdt', 'N/A')} kg/s")
    print("--------------------------\n")

    return condition

def main():
    """
    Reads binary log data, processes it, and generates plots.
    """
    try:
        parser = argparse.ArgumentParser(description='Plot data from a binary log file.')
        parser.add_argument('file', nargs='?', default=None, help='Optional path to a specific log file to plot.')
        parser.add_argument('--start', type=float, help='Start time for plotting in seconds (relative to the first sample).')
        parser.add_argument('--end', type=float, help='End time for plotting in seconds (relative to the first sample).')
        args = parser.parse_args()
        
        # Based on the structure defined in Logger.hpp
        # struct LogData {
        #     unsigned long t;            // 4 bytes + 4 padding
        #     double load_pos[3];          // 24 bytes
        #     double pointer_pos[3];       // 24 bytes
        #     double grip_force;           // 8 bytes
        #     double lugre_z[2];           // 16 bytes
        #     double lugre_dz[2];          // 16 bytes
        #     double lugre_v[2];           // 16 bytes
        #     double lugre_T;              // 8 bytes
        #     bool is_static_friction;    // 1 byte + 7 bytes padding
        #     double friction_force;       // 8 bytes
        #     double vibration_force;      // 8 bytes
        #     double mass;                 // 8 bytes
        # };
        # Total size = 144 bytes
        log_data_format = '<L 4x 3d 3d d 2d 2d 2d d ? 7x d d d'
        log_data_size = struct.calcsize(log_data_format)

        # --- File Selection ---
        if args.file:
            log_file_path = args.file
            print(f"Using provided log file: {log_file_path}")
        else:
            log_file_path = get_latest_log_file()

        if not log_file_path:
            print("No log file found.")
            print("You can specify a file path as an argument, e.g.: python drawGraph.py path/to/your/log.bin")
            return

        if not os.path.exists(log_file_path):
            print(f"Error: File not found at '{log_file_path}'")
            return

        # --- Data Extraction ---
        # Initialize lists to store the unpacked data
        data_lists = {
            't': [], 'load_pos': [], 'pointer_pos': [], 'grip_force': [],
            'lugre_z': [], 'lugre_dz': [], 'lugre_v': [], 'lugre_T': [],
            'is_static_friction': [], 'friction_force': [], 'vibration_force': [], 'mass': []
        }

        with open(log_file_path, 'rb') as f:
            # Read and print the Condition data at the beginning of the file
            condition_params = read_and_print_condition(f)
            if condition_params is None:
                return

            # Read LogData records until EOF
            while (chunk := f.read(log_data_size)):
                if len(chunk) < log_data_size:
                    print("Warning: Encountered an incomplete data chunk at the end of the file.")
                    continue

                unpacked_data = struct.unpack(log_data_format, chunk)
                
                # Map unpacked data to the dictionary
                i = 0
                data_lists['t'].append(unpacked_data[i]); i += 1
                data_lists['load_pos'].append(unpacked_data[i:i+3]); i += 3
                data_lists['pointer_pos'].append(unpacked_data[i:i+3]); i += 3
                data_lists['grip_force'].append(unpacked_data[i]); i += 1
                data_lists['lugre_z'].append(unpacked_data[i:i+2]); i += 2
                data_lists['lugre_dz'].append(unpacked_data[i:i+2]); i += 2
                data_lists['lugre_v'].append(unpacked_data[i:i+2]); i += 2
                data_lists['lugre_T'].append(unpacked_data[i]); i += 1
                data_lists['is_static_friction'].append(unpacked_data[i]); i += 1
                data_lists['friction_force'].append(unpacked_data[i]); i += 1
                data_lists['vibration_force'].append(unpacked_data[i]); i += 1
                data_lists['mass'].append(unpacked_data[i]); i += 1

        if not data_lists['t']:
            print("No data records were found in the log file.")
            return

        # Convert lists to numpy arrays for efficient processing
        for key, value in data_lists.items():
            data_lists[key] = np.array(value)

        t = data_lists['t'] / 1000.0 # Convert milliseconds to seconds
        if len(t) > 0:
            t = t - t[0] # Set the first sample's time to 0

        # --- Filter data based on command-line arguments ---
        start_index = 0
        end_index = len(t)

        if args.start is not None:
            start_index = np.searchsorted(t, args.start, side='left')

        if args.end is not None:
            end_index = np.searchsorted(t, args.end, side='right')
        
        if start_index >= end_index:
            print("Error: No data in the specified time range. Please check start_time and end_time.")
            return

        # Filter all data lists based on the calculated indices
        for key in data_lists:
            data_lists[key] = data_lists[key][start_index:end_index]
        
        # Recalculate time vector `t` after slicing
        t = data_lists['t'] / 1000.0
        if len(t) > 0:
            t = t - t[0]

        # --- Plotting ---
        # --- Call the custom plot function ---
        custom_data, custom_title = custom_plot_function(data_lists, t)
        
        plot_configs = [
            (data_lists['load_pos'][:, 0], 'Load Position X [m]'),
            (data_lists['load_pos'][:, 1], 'Load Position Y [m]'),
            (data_lists['load_pos'][:, 2], 'Load Position Z [m]'),
            (data_lists['pointer_pos'][:, 0], 'Pointer Position X [m]'),
            (data_lists['pointer_pos'][:, 1], 'Pointer Position Y [m]'),
            (data_lists['pointer_pos'][:, 2], 'Pointer Position Z [m]'),
            (data_lists['grip_force'], 'Grip Force [N]'),
            (data_lists['lugre_z'][:, 0], 'Lugre Z[0]'),
            (data_lists['lugre_z'][:, 1], 'Lugre Z[1]'),
            (data_lists['lugre_dz'][:, 0], 'Lugre dZ[0] [m/s]'),
            (data_lists['lugre_dz'][:, 1], 'Lugre dZ[1] [m/s]'),
            (data_lists['lugre_v'][:, 0], 'Lugre V[0] [m/s]'),
            (data_lists['lugre_v'][:, 1], 'Lugre V[1] [m/s]'),
            (data_lists['lugre_T'], 'Lugre T [s]'),
            (data_lists['is_static_friction'], 'Is Static Friction'),
            (data_lists['friction_force'], 'Friction Force [N]'),
            (data_lists['vibration_force'], 'Vibration Force [N]'),
            (data_lists['mass'], 'Object Mass [kg]')
        ]

        if custom_data is not None:
            plot_configs.append((custom_data, custom_title))

        num_subplots = len(plot_configs)

        if num_subplots == 0:
            print("No data to plot.")
            return

        fig, axs = plt.subplots(num_subplots, 1, figsize=(12, 2.5 * num_subplots), sharex=True)
        if num_subplots == 1:
            axs = [axs] # Make it iterable if there's only one subplot
        fig.suptitle('Log Data vs. Time', fontsize=16)

        for i, (data, title) in enumerate(plot_configs):
            ax = axs[i]
            if data.size > 0 and t.size == data.size:
                ax.plot(t, data)
            elif data.size > 0:
                print(f"Warning: Mismatch in length for '{title}'. Time has {t.size} points, data has {data.size} points. Skipping plot.")
            
            ax.set_title(title)
            ax.grid(True)
            if title == 'Is Static Friction':
                ax.set_ylim(-0.1, 1.1)

        if num_subplots > 0:
            axs[-1].set_xlabel('Time (t) [s]')

        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plot_filename = 'plot_combined.png'
        plt.savefig(plot_filename)
        print(f"Saved combined plot to '{plot_filename}'")
        print("\nTo display plots interactively, add 'plt.show()' at the end of the script.")
        # plt.show()

    except Exception as e:
        import traceback
        print(f"An unexpected error occurred: {e}", file=sys.stderr)
        traceback.print_exc()

if __name__ == '__main__':
    main()