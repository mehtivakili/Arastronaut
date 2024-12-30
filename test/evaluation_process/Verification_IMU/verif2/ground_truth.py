import csv
import time

# Open a CSV file to save the ground truth data
with open('ground_truth.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    
    # Write the header
    csvwriter.writerow(['timestamp_ns', 'user_input'])

    try:
        while True:
            # Get user input
            user_input = input("Enter a value for ground truth (or type 'exit' to quit): ")
            
            # Break the loop if user wants to quit
            if user_input.lower() == 'exit':
                print("Exiting program...")
                break
            
            # Get system timestamp in nanoseconds
            system_timestamp_ns = int(time.time_ns())
            
            # Save the timestamp and user input to the CSV file
            csvwriter.writerow([system_timestamp_ns, user_input])
            csvfile.flush()  # Ensure data is written to the file immediately

            # Print confirmation
            print(f"Saved input '{user_input}' at timestamp {system_timestamp_ns}")

    except KeyboardInterrupt:
        print("Program interrupted")
