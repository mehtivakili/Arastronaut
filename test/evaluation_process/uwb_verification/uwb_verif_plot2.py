import sys
import pandas as pd
import matplotlib.pyplot as plt

def plot_data(uwb_file_path):
    try:
        # Read UWB data from the CSV file
        uwb_data = pd.read_csv(uwb_file_path)

        # Check if data is read correctly
        if uwb_data.empty:
            raise ValueError(f"UWB data file '{uwb_file_path}' is empty or cannot be read.")

        # Plot the data
        fig, ax = plt.subplots(figsize=(10, 6))

        # Plot distance over time
        ax.plot(uwb_data['Time (s)'], uwb_data['Distance (m)'], label='Distance')

        ax.set_title('UWB Distance vs Time')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance (m)')
        ax.legend()

        # Save plot to SVG file
        plot_file_path = 'uwb_plot.svg'
        plt.tight_layout()
        plt.savefig(plot_file_path, format='svg')
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
    # Provide the correct path to your CSV file
    uwb_file_path = 'time_distance_data1.csv'

    # Call the function to plot the data
    plot_file_path = plot_data(uwb_file_path)
    print(f"Plot saved as: {plot_file_path}")
