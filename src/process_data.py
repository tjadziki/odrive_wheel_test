"""
Data post-processing utility for ODrive and Arduino test data.

This script processes the raw CSV data from combined ODrive and Arduino tests:
1. Properly aligns and interpolates data from both sources
2. Applies gravity compensation to accelerometer data 
3. Exports to both CSV and Excel formats
"""

import pandas as pd
import numpy as np
import os
import glob
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

def load_data(filename):
    """
    Load test data from CSV and perform initial cleaning.
    
    Args:
        filename (str): Path to the CSV file
        
    Returns:
        pd.DataFrame: Processed and cleaned DataFrame
    """
    print(f"Loading data from {filename}...")
    
    # Load the CSV data
    df = pd.read_csv(filename)
    
    # Convert timestamps to numeric
    df['timestamp'] = pd.to_numeric(df['timestamp'], errors='coerce')
    
    # Split into ODrive and Arduino data
    odrive_data = df[df['source'] == 'odrive'].copy()
    arduino_data = df[df['source'] == 'arduino'].copy()
    
    print(f"Found {len(odrive_data)} ODrive records and {len(arduino_data)} Arduino records")
    
    return df, odrive_data, arduino_data

def align_data(odrive_data, arduino_data):
    """
    Align ODrive and Arduino data by timestamp, creating a coherent timeline.
    
    Args:
        odrive_data (pd.DataFrame): ODrive data
        arduino_data (pd.DataFrame): Arduino data
        
    Returns:
        pd.DataFrame: Aligned DataFrame with both data sources
    """
    print("Aligning ODrive and Arduino data...")
    
    # If one of the dataframes is empty or has no real data, just use the other one
    if len(odrive_data) == 0 or odrive_data.isnull().all().all():
        print("No valid ODrive data found - using Arduino data only")
        result = arduino_data.copy()
        result['source'] = 'arduino'
        return result
    
    if len(arduino_data) == 0 or arduino_data.isnull().all().all():
        print("No valid Arduino data found - using ODrive data only")
        result = odrive_data.copy()
        result['source'] = 'odrive'
        return result
    
    # Ensure timestamps are properly formatted as floats
    try:
        odrive_data['timestamp'] = odrive_data['timestamp'].astype(float)
    except (ValueError, TypeError):
        print("Warning: ODrive timestamps not numeric - using sequential values")
        odrive_data['timestamp'] = np.arange(len(odrive_data))
    
    try:
        arduino_data['timestamp'] = arduino_data['timestamp'].astype(float)
    except (ValueError, TypeError):
        print("Warning: Arduino timestamps not numeric - using sequential values")
        arduino_data['timestamp'] = np.arange(len(arduino_data))
    
    # Determine the common time range
    min_timestamp = max(
        odrive_data['timestamp'].min() if not odrive_data.empty else float('inf'),
        arduino_data['timestamp'].min() if not arduino_data.empty else float('inf')
    )
    
    max_timestamp = min(
        odrive_data['timestamp'].max() if not odrive_data.empty else float('-inf'),
        arduino_data['timestamp'].max() if not arduino_data.empty else float('-inf')
    )
    
    # Safeguard against invalid time ranges
    if min_timestamp >= max_timestamp:
        print("Warning: Data sources have non-overlapping timestamps")
        print(f"ODrive timestamps: {odrive_data['timestamp'].min()} to {odrive_data['timestamp'].max()}")
        print(f"Arduino timestamps: {arduino_data['timestamp'].min()} to {arduino_data['timestamp'].max()}")
        
        # Create a unified time scale for non-overlapping data
        all_timestamps = np.concatenate([
            odrive_data['timestamp'].values,
            arduino_data['timestamp'].values
        ])
        min_timestamp = all_timestamps.min()
        max_timestamp = all_timestamps.max()
        print(f"Using combined timestamp range: {min_timestamp} to {max_timestamp}")
    
    # Create a regular time grid for interpolation (100 points or actual sample count, whichever is larger)
    grid_points = max(100, len(odrive_data) + len(arduino_data))
    try:
        time_points = np.linspace(min_timestamp, max_timestamp, grid_points)
    except Exception as e:
        print(f"Error creating time points: {e}")
        # Fallback option: create sequential timestamps
        time_points = np.arange(grid_points)
        # Update the original dataframes with these sequential timestamps
        if len(odrive_data) > 0:
            odrive_idx = np.round(np.linspace(0, grid_points-1, len(odrive_data))).astype(int)
            odrive_data['timestamp'] = time_points[odrive_idx]
        if len(arduino_data) > 0:
            arduino_idx = np.round(np.linspace(0, grid_points-1, len(arduino_data))).astype(int)
            arduino_data['timestamp'] = time_points[arduino_idx]
    
    # Interpolate ODrive data onto the time grid
    odrive_numeric = odrive_data.select_dtypes(include=['number'])
    odrive_interp = pd.DataFrame(index=range(len(time_points)))
    odrive_interp['timestamp'] = time_points
    
    for col in odrive_numeric.columns:
        if col != 'timestamp' and col in odrive_data.columns and not odrive_data[col].isnull().all():
            try:
                # Use linear interpolation for ODrive data
                odrive_interp[col] = np.interp(
                    time_points, 
                    odrive_data['timestamp'],
                    odrive_data[col].fillna(0),  # Fill NaN with zeros for interpolation
                    left=np.nan, right=np.nan
                )
            except Exception as e:
                print(f"Error interpolating ODrive column {col}: {e}")
                # Skip this column
                continue
    
    # Interpolate Arduino data onto the time grid
    arduino_numeric = arduino_data.select_dtypes(include=['number'])
    arduino_interp = pd.DataFrame(index=range(len(time_points)))
    arduino_interp['timestamp'] = time_points
    
    for col in arduino_numeric.columns:
        if col != 'timestamp' and col in arduino_data.columns and not arduino_data[col].isnull().all():
            try:
                # Use linear interpolation for Arduino data
                arduino_interp[col] = np.interp(
                    time_points, 
                    arduino_data['timestamp'],
                    arduino_data[col].fillna(0),  # Fill NaN with zeros for interpolation
                    left=np.nan, right=np.nan
                )
            except Exception as e:
                print(f"Error interpolating Arduino column {col}: {e}")
                # Skip this column
                continue
    
    # Merge the interpolated datasets
    aligned_df = pd.merge(odrive_interp, arduino_interp, on='timestamp', how='outer', suffixes=('_odrive', '_arduino'))
    
    # Fill non-numeric columns
    aligned_df['source'] = 'combined'
    
    # Keep only rows where at least some data is available
    aligned_df = aligned_df.dropna(how='all')
    
    print(f"Created aligned dataset with {len(aligned_df)} records")
    
    return aligned_df

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
        # IMPORTANT: The order ('xyz', 'zyx', etc.) depends on how the Madgwick
        # library defines Roll, Pitch, Yaw. Check its documentation.
        # 'xyz' means apply Yaw, then Pitch, then Roll intrinsic rotations.
        # 'zyx' is also common (Yaw, Pitch, Roll). Let's assume 'zyx' for now.
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
    """
    Process a single data file, performing alignment and gravity compensation.
    
    Args:
        filepath (str): Path to the CSV file to process
        
    Returns:
        str: Path to the output Excel file
    """
    print(f"\nProcessing: {filepath}")
    
    try:
        # Step 1: Load and clean the data
        df, odrive_data, arduino_data = load_data(filepath)
        
        if len(arduino_data) == 0:
            print("Warning: No Arduino data found - gravity compensation not possible")
            return None
            
        # Step 2: Align the data by timestamp
        aligned_df = align_data(odrive_data, arduino_data)
        
        # Check if we have the necessary columns for gravity compensation
        required_columns = ['ax', 'ay', 'az', 'roll', 'pitch', 'yaw']
        missing_columns = [col for col in required_columns if col not in aligned_df.columns]
        
        if missing_columns:
            print(f"Warning: Missing required columns for gravity compensation: {missing_columns}")
            print("Skipping gravity compensation")
        else:
            # Step 3: Apply gravity compensation
            print("Applying gravity compensation...")
            # Drop rows with NaN values in required columns before applying gravity compensation
            valid_rows = aligned_df.dropna(subset=required_columns)
            
            if len(valid_rows) > 0:
                linear_accel = valid_rows.apply(compensate_gravity_euler, axis=1)
                
                # Create a new aligned_df with gravity compensation applied
                for col in linear_accel.columns:
                    aligned_df.loc[valid_rows.index, col] = linear_accel[col]
                
                # Calculate total linear acceleration magnitude (where possible)
                if all(col in aligned_df.columns for col in ['lin_ax', 'lin_ay', 'lin_az']):
                    print("Calculating linear acceleration magnitude...")
                    aligned_df['lin_accel_magnitude'] = np.sqrt(
                        aligned_df['lin_ax']**2 + 
                        aligned_df['lin_ay']**2 + 
                        aligned_df['lin_az']**2
                    )
            else:
                print("No valid rows found for gravity compensation")
        
        # Generate output filenames
        base_path = os.path.splitext(filepath)[0]
        output_csv = f"{base_path}_processed.csv"
        output_excel = f"{base_path}_processed.xlsx"
        
        # Save the processed data to CSV first
        aligned_df.to_csv(output_csv, index=False)
        print(f"Processed data saved to {output_csv}")
        
        # Create Excel output with proper formatting
        try:
            print("Creating Excel report...")
            with pd.ExcelWriter(output_excel, engine='openpyxl') as writer:
                # Add the main processed data
                aligned_df.to_excel(writer, sheet_name='Processed Data', index=False)
                
                # Add a summary sheet with useful test metrics
                summary_data = {
                    'Metric': [
                        'Test Name',
                        'Date and Time',
                        'Total Duration (s)',
                        'Number of Samples',
                        'Arduino Samples',
                        'ODrive Samples'
                    ],
                    'Value': [
                        os.path.basename(filepath).split('_')[2],
                        ' '.join(os.path.basename(filepath).split('_')[3:]).replace('.csv', ''),
                        f"{aligned_df['timestamp'].max() - aligned_df['timestamp'].min():.2f}",
                        len(aligned_df),
                        len(arduino_data),
                        len(odrive_data)
                    ]
                }
                
                # Add position data metrics if available
                if 'position' in aligned_df.columns and not aligned_df['position'].isnull().all():
                    summary_data['Metric'].extend([
                        'Max Position (turns)',
                        'Min Position (turns)'
                    ])
                    summary_data['Value'].extend([
                        f"{aligned_df['position'].max():.3f}",
                        f"{aligned_df['position'].min():.3f}"
                    ])
                
                # Add velocity data metrics if available
                if 'velocity' in aligned_df.columns and not aligned_df['velocity'].isnull().all():
                    summary_data['Metric'].extend([
                        'Max Velocity (turns/s)',
                        'Average Velocity (turns/s)'
                    ])
                    summary_data['Value'].extend([
                        f"{aligned_df['velocity'].max():.3f}",
                        f"{aligned_df['velocity'].mean():.3f}"
                    ])
                
                # Add linear acceleration metrics if available
                if 'lin_accel_magnitude' in aligned_df.columns and not aligned_df['lin_accel_magnitude'].isnull().all():
                    summary_data['Metric'].extend([
                        'Max Linear Acceleration (g)',
                        'Average Linear Acceleration (g)',
                        'Standard Deviation of Linear Acceleration (g)'
                    ])
                    summary_data['Value'].extend([
                        f"{aligned_df['lin_accel_magnitude'].max():.3f}",
                        f"{aligned_df['lin_accel_magnitude'].mean():.3f}",
                        f"{aligned_df['lin_accel_magnitude'].std():.3f}"
                    ])
                
                # Create and write summary dataframe
                summary_df = pd.DataFrame(summary_data)
                summary_df.to_excel(writer, sheet_name='Summary', index=False)
            
            print(f"Excel report saved to {output_excel}")
            return output_excel
        
        except Exception as e:
            print(f"Warning: Could not create Excel file: {e}")
            print(f"CSV data was still saved to {output_csv}")
            return output_csv
            
    except Exception as e:
        print(f"Error processing {filepath}: {e}")
        import traceback
        traceback.print_exc()
        return None

def process_latest_file():
    """
    Find and process the most recent data file in the data directory.
    
    Returns:
        str: Path to the processed Excel file, or None if not found
    """
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
    
    # Process the file
    return process_file(latest_file)

def process_specific_file(filename):
    """
    Process a specific data file.
    
    Args:
        filename (str): Name of the file to process (with or without path)
        
    Returns:
        str: Path to the processed Excel file, or None if not found
    """
    # Check if path is provided
    if os.path.dirname(filename):
        filepath = filename
    else:
        # Assume file is in the data directory
        filepath = os.path.join("data", filename)
    
    if not os.path.exists(filepath):
        print(f"Error: File '{filepath}' not found")
        return None
    
    # Process the file
    return process_file(filepath)

def main():
    """Main function to process test data files."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Process ODrive+Arduino test data files')
    parser.add_argument('-f', '--file', help='Specific file to process')
    parser.add_argument('-a', '--all', action='store_true', help='Process all files in the data directory')
    args = parser.parse_args()
    
    if args.file:
        output_file = process_specific_file(args.file)
        if output_file:
            print(f"\nSuccessfully processed file. Output: {output_file}")
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
        
        processed_files = []
        for file in csv_files:
            output_file = process_file(file)
            if output_file:
                processed_files.append(output_file)
                
        print(f"\nSuccessfully processed {len(processed_files)} files")
    else:
        # Process the most recent file by default
        output_file = process_latest_file()
        if output_file:
            print(f"\nSuccessfully processed latest file. Output: {output_file}")
    
if __name__ == "__main__":
    main()
