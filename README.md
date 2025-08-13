![CRoCs Logo](assets/logo.png)
# SwiFT;

**SwiFT** is a lightweight tool for the calibration and data acquisition of 6-axis force/torque sensors, aimed at simplifying and optimizing experimental workflows. This project was developed at the Department of Orthodontics, Pusan National University Dental Hospital.

by Hyeonjin Jo\* and Yongil Kim<sup>+</sup>   

\* Graduate Student @School of Dentistry, Pusan National University, Republic of Korea  
<sup>+</sup> Professor @Department of Orthodontics, Pusan National University Dental Hospital, Republic of Korea

-----

## Prerequisites

1.  **Download and Install Miniconda**

    You have two options for downloading Miniconda.

      * **Option A: Direct Download (Recommended)**
        Go to the [Miniconda repository](https://repo.anaconda.com/miniconda/) and download the appropriate installer for your operating system (`.exe` for Windows, `.sh` for Linux, `.pkg` for macOS).

      * **Option B: Using the Command Line (Windows)**
        Alternatively, open your terminal and run the following command to download the installer for Windows.

        ```bash
        curl https://repo.anaconda.com/miniconda/Miniconda3-latest-Windows-x86_64.exe --output Miniconda3-installer.exe
        ```

    > After downloading, **run the installer** and follow the on-screen instructions to complete the setup.

2.  **Set Up Conda Environment**
    Create and activate a new Conda environment named `SwiFT`.

    ```bash
    conda init
    conda create -n SwiFT python=3.12
    conda activate SwiFT
    ```

3.  **Install Dependencies**
    Install all the required Python packages from the `requirements.txt` file.

    ```bash
    pip install -r requirements.txt
    ```

-----

## Usage

1.  **Make the Script Executable** (Only required for the first time on Linux/macOS)
    ```bash
    chmod +x run.sh
    ```

2.  **Run the Application**
    ```bash
    ./run.sh
    ```

3.  **Configuration Parameters**
    ```python
    SAVE_PATH = "6072/"
    BITRATE = 1000000
    SAMPLING_RATE = 0.1  # seconds (10 Hz)
    NUM_FLOATING_POINT = 2
    CALIBRATION_TIME = 3
    MOVING_AVERAGE_TERM = 5
    RECORD_BUFFER = 1
    RECORD_TIME = 5
    ```
    
4.  **Manual**
    1. Record Start : press R
    2. Save Results : press S *(Note: Data is saved automatically after recording finishes)*
    3. Exit the program : press F

5. **Operation Flow**
    - When **Record** starts:
        1. **Calibration** runs for `CALIBRATION_TIME` seconds.
        2. After calibration, press **SPACE** to start recording.  
           At this stage, seat the clear aligner over the experimental apparatus before proceeding.
        3. **Recording** then runs for `RECORD_TIME` seconds.
        4. To ensure the full `RECORD_TIME` is captured, there is a delay of `RECORD_BUFFER` seconds before and after the recording period.
    - **Example**  
      If `CALIBRATION_TIME = 3`, `RECORD_TIME = 5`, and `RECORD_BUFFER = 1`,  
      the sequence will be:  
      **Calibration (3s) → [Seat clear aligner over the experimental apparatus → Press SPACE] → Delay (1s) → Recording (5s) → Delay (1s)**
