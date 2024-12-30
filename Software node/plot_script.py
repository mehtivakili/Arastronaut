import sys
import pandas as pd
import matplotlib.pyplot as plt

def plot_data(acc_file_path, gyro_file_path):
    try:
        # Read accelerometer data
        acc_data = pd.read_csv(acc_file_path, delim_whitespace=True, header=None, names=['time', 'acc_x', 'acc_y', 'acc_z'])
        # Read gyroscope data
        gyro_data = pd.read_csv(gyro_file_path, delim_whitespace=True, header=None, names=['time', 'gyro_x', 'gyro_y', 'gyro_z'])

        # Check if data is read correctly
        if acc_data.empty:
            raise ValueError(f"Accelerometer data file '{acc_file_path}' is empty or cannot be read.")
        if gyro_data.empty:
            raise ValueError(f"Gyroscope data file '{gyro_file_path}' is empty or cannot be read.")

        # Plot data
        fig, axs = plt.subplots(2, 1, figsize=(10, 6))

        # Plot accelerometer data
        axs[0].plot(acc_data['time'], acc_data['acc_x'], label='Acc X')
        axs[0].plot(acc_data['time'], acc_data['acc_y'], label='Acc Y')
        axs[0].plot(acc_data['time'], acc_data['acc_z'], label='Acc Z')
        axs[0].set_title('Accelerometer Data')
        axs[0].set_xlabel('Time')
        axs[0].set_ylabel('Acceleration')
        axs[0].legend()

        # Plot gyroscope data
        axs[1].plot(gyro_data['time'], gyro_data['gyro_x'], label='Gyro X')
        axs[1].plot(gyro_data['time'], gyro_data['gyro_y'], label='Gyro Y')
        axs[1].plot(gyro_data['time'], gyro_data['gyro_z'], label='Gyro Z')
        axs[1].set_title('Gyroscope Data')
        axs[1].set_xlabel('Time')
        axs[1].set_ylabel('Angular Velocity')
        axs[1].legend()

        # Save plot to file
        plot_file_path = 'plot.png'
        plt.tight_layout()
        plt.savefig(plot_file_path)
        plt.close()
        return plot_file_path
    except FileNotFoundError as e:
        print(f'Error: {str(e)}', file=sys.stderr)
        sys.exit(1)
    except ValueError as e:
        print(f'Error: {str(e)}', file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f'Error: {str(e)}', file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: plot_script.py <acc_file_path> <gyro_file_path>", file=sys.stderr)
        sys.exit(1)

    acc_file_path = sys.argv[1]
    gyro_file_path = sys.argv[2]
    plot_file_path = plot_data(acc_file_path, gyro_file_path)
    print(plot_file_path)
