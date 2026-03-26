import pandas as pd
import numpy as np

# Define the file names
GROUND_TRUTH_FILE = "trajectories_intersection_64_withoutQuanti.csv"
FLOAT32_FILE = "trajectories_intersection_32_withoutQuanti.csv"
POSIT32_FILE = "trajectories_intersection_32_withoutQuanti_posit.csv"

def load_and_preprocess(filename):
    """Loads the CSV and creates a safe alignment index."""
    try:
        df = pd.read_csv(filename)
        # Create a sequential step counter for each vehicle to safely align rows
        df["step"] = df.groupby("vehicle_id").cumcount()
        # Set index to vehicle_id and step for guaranteed 1-to-1 matching
        df.set_index(["vehicle_id", "step"], inplace=True)
        return df
    except FileNotFoundError:
        print(f"[ERROR] File not found: {filename}")
        return None

def calculate_metrics(df_gt, df_comp, label):
    """Calculates Euclidean error and prints statistics."""
    if df_comp is None:
        return

    # Inner join guarantees we only compare exact matching steps/vehicles
    aligned = df_comp.join(df_gt, lsuffix='_comp', rsuffix='_gt', how='inner')
    
    # Calculate Euclidean distance error: sqrt(dx^2 + dy^2)
    dx = aligned["x_comp"] - aligned["x_gt"]
    dy = aligned["y_comp"] - aligned["y_gt"]
    errors = np.sqrt(dx**2 + dy**2)

    # Print the statistical summary
    print(f"=== Precision Analysis: {label} vs 64-bit IEEE Baseline ===")
    print(f"Data Points Compared: {len(errors)}")
    print(f"Mean Error:   {errors.mean():.8e} meters")
    print(f"Median Error: {errors.median():.8e} meters")
    print(f"Max Error:    {errors.max():.8e} meters")
    print(f"Min Error:    {errors.min():.8e} meters")
    print(f"Std Dev:      {errors.std():.8e} meters")
    print("=" * 60 + "\n")

def main():
    print("\nLoading trajectory data...")
    df_gt = load_and_preprocess(GROUND_TRUTH_FILE)
    
    if df_gt is None:
        print("\n[STOP] Ground truth data (64-bit) is required to perform the analysis.")
        print("Please run your C++ program with REAL_BITS=64 to generate the baseline.")
        return

    df_f32 = load_and_preprocess(FLOAT32_FILE)
    df_p32 = load_and_preprocess(POSIT32_FILE)

    print("Data loaded successfully. Calculating metrics...\n")
    
    calculate_metrics(df_gt, df_f32, "32-bit IEEE Float")
    calculate_metrics(df_gt, df_p32, "32-bit Posit")

if __name__ == "__main__":
    main()
