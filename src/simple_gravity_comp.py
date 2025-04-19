"""
Simple gravity compensation script for Arduino accelerometer data.

This script processes the Arduino accelerometer data and applies gravity compensation,
outputting the results to both CSV and Excel formats for easy analysis.
"""

import pandas as pd
import numpy as np
import os
import glob
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

def compensate_gravity_euler(row):
    """
    Calculates linear acceleration by removing gravity using Euler angle orientation.

    Args:
        row (pd.Series): A row of the DataFrame containing timestamp,
                         ax, ay, az, roll, pitch, yaw.

    Returns:
        pd.Series: Series containing linear acceleration components lin_ax, lin_ay, lin_az.
                   Returns NaNs if angle data is invalid.
    """
    try:
        # Raw acceleration vector (assuming units of 'g')
        accel_raw = np.array([row['ax'], row['ay'], row['az']])

        # Earth gravity vector (assuming Z is up, magnitude 1g)
        gravity_earth = np.array([0.0, 0.0, 1.0])

        # --- Get Euler Angles (convert degrees to radians) ---
        # Check for invalid angle data
        if pd.isna(row['roll']) or pd.isna(row['pitch']) or pd.isna(row['yaw']):
             print(f"Warning: Invalid angles at timestamp {row.get('timestamp', 'N/A')}")
             return pd.Series({'lin_ax': np.nan, 'lin_ay': np.nan, 'lin_az': np.nan})

        roll_rad = np.radians(row['roll'])
        pitch_rad = np.radians(row['pitch'])
        yaw_rad = np.radians(row['yaw'])

        # --- Create Rotation Object ---
        # 'zyx' rotation order (Yaw, Pitch, Roll) is common for IMU sensors
        try:
             rotation = R.from_euler('zyx', [yaw_rad, pitch_rad, roll_rad])
        except Exception as e:
             print(f"Error creating rotation object at timestamp {row.get('timestamp', 'N/A')}: {e}")
             return pd.Series({'lin_ax': np.nan, 'lin_ay': np.nan, 'lin_az': np.nan})

        # --- Rotate Gravity Vector into Sensor Frame ---
        # Apply the inverse rotation to the Earth gravity vector
        gravity_sensor_frame = rotation.apply(gravity_earth, inverse=True)

        # --- Calculate Linear Acceleration ---
        accel_linear = accel_raw - gravity_sensor_frame

        return pd.Series({
            'lin_ax': accel_linear[0],
            'lin_ay': accel_linear[1],
            'lin_az': accel_linear[2]
        })

    except Exception as e:
        print(f"Error processing row: {e}")
        # Return NaNs or default values on error
        return pd.Series({'lin_ax': np.nan, 'lin_ay': np.nan, 'lin_az': np.nan})

def process_file(filepath):
    """Process a single data file and apply gravity compensation to Arduino data."""
    print(f"\nProcessing: {filepath}")
    
    try:
        # Load the CSV data
        df = pd.read_csv(filepath)
        print(f"Loaded file with {len(df)} records")
        
        # Filter Arduino data only
        arduino_data = df[df['source'] == 'arduino'].copy()
        print(f"Found {len(arduino_data)} Arduino records")
        
        if len(arduino_data) == 0:
            print("No Arduino data found. Cannot proceed with gravity compensation.")
            return None
            
        # Ensure all required columns exist
        required_columns = ['ax', 'ay', 'az', 'roll', 'pitch', 'yaw']
        for col in required_columns:
            if col not in arduino_data.columns:
                print(f"Missing required column: {col}")
                return None
        
        # Apply gravity compensation
        print("Applying gravity compensation...")
        valid_rows = arduino_data.dropna(subset=required_columns)
        
        if len(valid_rows) == 0:
            print("No valid rows found for gravity compensation")
            return None
            
        linear_accel = valid_rows.apply(compensate_gravity_euler, axis=1)
        
        # Add linear acceleration to the original data
        for col in linear_accel.columns:
            arduino_data.loc[valid_rows.index, col] = linear_accel[col]
        
        # Calculate total linear acceleration magnitude
        print("Calculating linear acceleration magnitude...")
        arduino_data['lin_accel_magnitude'] = np.sqrt(
            arduino_data['lin_ax']**2 + 
            arduino_data['lin_ay']**2 + 
            arduino_data['lin_az']**2
        )
        
        # Generate output filenames
        base_path = os.path.splitext(filepath)[0]
        output_csv = f"{base_path}_gravity_comp.csv"
        output_excel = f"{base_path}_gravity_comp.xlsx"
        
        # Save the processed data to CSV
        arduino_data.to_csv(output_csv, index=False)
        print(f"Processed Arduino data saved to {output_csv}")
        
        # Create an Excel workbook with useful sheets and basic visualizations
        print("Creating Excel report...")
        with pd.ExcelWriter(output_excel, engine='openpyxl') as writer:
            # Add the processed Arduino data
            arduino_data.to_excel(writer, sheet_name='Processed Arduino Data', index=False)
            
            # Add a summary sheet
            summary = pd.DataFrame({
                'Metric': [
                    'Test Name',
                    'Date and Time',
                    'Number of Samples',
                    'Min Timestamp',
                    'Max Timestamp',
                    'Test Duration (s)',
                    'Max Linear Acceleration (g)',
                    'Mean Linear Acceleration (g)',
                    'Standard Deviation (g)'
                ],
                'Value': [
                    os.path.basename(filepath).split('_')[2],
                    ' '.join(os.path.basename(filepath).split('_')[3:]).replace('.csv', ''),
                    len(arduino_data),
                    f"{arduino_data['timestamp'].min():.2f}",
                    f"{arduino_data['timestamp'].max():.2f}",
                    f"{arduino_data['timestamp'].max() - arduino_data['timestamp'].min():.2f}",
                    f"{arduino_data['lin_accel_magnitude'].max():.4f}",
                    f"{arduino_data['lin_accel_magnitude'].mean():.4f}",
                    f"{arduino_data['lin_accel_magnitude'].std():.4f}"
                ]
            })
            summary.to_excel(writer, sheet_name='Summary', index=False)
        
        print(f"Excel report saved to {output_excel}")
        return output_excel
        
    except Exception as e:
        print(f"Error processing {filepath}: {e}")
        import traceback
        traceback.print_exc()
        return None

def process_latest_file():
    """Find and process the most recent test data file."""
    data_dir = "data"
    if not os.path.exists(data_dir):
        print(f"Error: Data directory '{data_dir}' not found")
        return None
        
    # Find all CSV files in the data directory
    csv_files = glob.glob(os.path.join(data_dir, "wheel_test_*.csv"))
    
    if not csv_files:
        print("No test data files found")
        return None
        
    # Get the most recent file based on modification time
    latest_file = max(csv_files, key=os.path.getmtime)
    
    print(f"Found latest file: {latest_file}")
    return process_file(latest_file)

def main():
    """Main function to process test data files."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Process Arduino test data and apply gravity compensation')
    parser.add_argument('-f', '--file', help='Specific file to process')
    parser.add_argument('-a', '--all', action='store_true', help='Process all files in the data directory')
    args = parser.parse_args()
    
    if args.file:
        filepath = args.file
        if not os.path.isabs(filepath):
            filepath = os.path.join("data", filepath)
        process_file(filepath)
    elif args.all:
        data_dir = "data"
        if not os.path.exists(data_dir):
            print(f"Error: Data directory '{data_dir}' not found")
            return
            
        csv_files = glob.glob(os.path.join(data_dir, "wheel_test_*.csv"))
        
        if not csv_files:
            print("No test data files found")
            return
            
        print(f"Found {len(csv_files)} files to process")
        
        for file in csv_files:
            process_file(file)
    else:
        # Process the most recent file by default
        process_latest_file()
    
if __name__ == "__main__":
    main()
