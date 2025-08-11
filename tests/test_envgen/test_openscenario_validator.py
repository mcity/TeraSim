import os
import sys
import logging
import subprocess
import time
import glob

# Set up logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Configure esmini path and environment
ESMINI_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "esmini", "bin"
)
ESMINI_PATH = os.path.join(ESMINI_DIR, "esmini")

# Set up environment variables
os.environ["LD_LIBRARY_PATH"] = (
    os.environ.get("LD_LIBRARY_PATH", "") + os.pathsep + ESMINI_DIR
)


def check_esmini_installation():
    """Check if esmini is installed and accessible"""
    try:
        # Ensure executable permissions
        subprocess.run(["chmod", "+x", ESMINI_PATH], check=True)

        result = subprocess.run(
            [ESMINI_PATH, "--version"], capture_output=True, text=True
        )
        # Check if version information is in stdout, regardless of return code
        if "esmini GIT" in result.stdout:
            logger.info(f"Found esmini version: {result.stdout.strip()}")
            return True
        else:
            logger.error("esmini did not return version information")
            logger.error(f"stdout: {result.stdout}")
            logger.error(f"stderr: {result.stderr}")
            return False
    except FileNotFoundError:
        logger.error(f"esmini not found at: {ESMINI_PATH}")
        return False
    except subprocess.CalledProcessError as e:
        logger.error(f"Error setting permissions: {str(e)}")
        return False


def validate_openscenario_file(file_path, timeout=30):
    """
    Validate an OpenSCENARIO file using esmini
    Returns: (bool, str) - (success, message)
    """
    try:
        start_time = time.time()

        # 获取场景目录路径
        scenario_dir = os.path.dirname(file_path)

        # 检查OpenDRIVE文件是否存在
        xodr_file = os.path.join(scenario_dir, "map.xodr")
        if not os.path.exists(xodr_file):
            logger.error(f"OpenDRIVE file not found: {xodr_file}")
            return False, "Missing OpenDRIVE file (map.xodr)"

        # 修改命令，添加更多参数以获取详细输出
        cmd = [
            ESMINI_PATH,
            "--osc",
            file_path,  # 使用--osc而不是直接传递文件路径
            "--odr",
            xodr_file,  # 指定OpenDRIVE文件路径
            "--validate",  # 验证模式
            "--headless",  # 无界面模式
            "--disable-controllers",  # 禁用控制器以加快验证
            "--disable-signals",  # 禁用交通信号灯验证
            "--tolerance",
            "0.1",  # 设置位置验证容差为0.1米
            "--logfile",
            os.path.join(scenario_dir, "validation.log"),  # 输出日志到场景目录
        ]

        logger.info(f"Validating file: {file_path}")
        logger.info(f"Using OpenDRIVE file: {xodr_file}")
        logger.info(f"Running command: {' '.join(cmd)}")

        # Add environment info for debugging
        logger.info(
            f"Current LD_LIBRARY_PATH: {os.environ.get('LD_LIBRARY_PATH', 'Not set')}"
        )
        logger.info(f"Current working directory: {os.getcwd()}")

        # 切换到场景目录运行命令
        original_dir = os.getcwd()
        os.chdir(scenario_dir)

        try:
            process = subprocess.run(
                cmd, capture_output=True, text=True, timeout=timeout
            )
        finally:
            # 恢复原始工作目录
            os.chdir(original_dir)

        validation_time = time.time() - start_time

        # Always log the complete output
        logger.info("Process stdout:")
        for line in process.stdout.splitlines():
            logger.info(f"  {line}")

        logger.info("Process stderr:")
        for line in process.stderr.splitlines():
            logger.info(f"  {line}")

        logger.info(f"Process return code: {process.returncode}")

        # Try to read the validation log file
        log_file = os.path.join(scenario_dir, "validation.log")
        try:
            if os.path.exists(log_file):
                logger.info("Validation log file contents:")
                with open(log_file, "r") as f:
                    for line in f:
                        logger.info(f"  {line.strip()}")
        except Exception as e:
            logger.error(f"Error reading validation log: {str(e)}")

        if process.returncode == 0:
            return True, f"Validation successful (took {validation_time:.2f}s)"
        else:
            error_msg = process.stderr if process.stderr else "Unknown error"
            logger.error(f"Validation failed with return code {process.returncode}")
            return False, f"Validation failed: {error_msg}"

    except subprocess.TimeoutExpired:
        logger.error(f"Command timed out after {timeout} seconds")
        return False, f"Validation timed out after {timeout} seconds"
    except Exception as e:
        logger.error(f"Unexpected error during validation: {str(e)}")
        return False, f"Validation error: {str(e)}"


def visualize_openscenario_file(file_path, timeout=30):
    """
    Visualize an OpenSCENARIO file using esmini
    Returns: (bool, str) - (success, message)
    """
    try:
        # 获取场景目录路径
        scenario_dir = os.path.dirname(file_path)

        # 检查OpenDRIVE文件是否存在
        xodr_file = os.path.join(scenario_dir, "map.xodr")
        if not os.path.exists(xodr_file):
            logger.error(f"OpenDRIVE file not found: {xodr_file}")
            return False, "Missing OpenDRIVE file (map.xodr)"

        # 修改命令，添加可视化相关参数
        cmd = [
            ESMINI_PATH,
            "--osc",
            file_path,  # 使用--osc而不是直接传递文件路径
            "--odr",
            xodr_file,  # 指定OpenDRIVE文件路径
            "--window",
            "60",
            "60",
            "800",
            "600",  # 设置窗口位置和大小
            "--disable-controllers",  # 禁用控制器以加快加载
            "--disable-signals",  # 禁用交通信号灯验证
            "--tolerance",
            "0.1",  # 设置位置验证容差为0.1米
        ]

        logger.info(f"Visualizing file: {file_path}")
        logger.info(f"Using OpenDRIVE file: {xodr_file}")
        logger.info(f"Running command: {' '.join(cmd)}")

        # 切换到场景目录运行命令
        original_dir = os.getcwd()
        os.chdir(scenario_dir)

        try:
            process = subprocess.run(cmd, timeout=timeout)
            if process.returncode == 0:
                return True, "Visualization completed successfully"
            else:
                return (
                    False,
                    f"Visualization failed with return code {process.returncode}",
                )
        finally:
            # 恢复原始工作目录
            os.chdir(original_dir)

    except subprocess.TimeoutExpired:
        return False, f"Visualization timed out after {timeout} seconds"
    except Exception as e:
        return False, f"Visualization error: {str(e)}"


def main():
    # Check esmini installation first
    if not check_esmini_installation():
        logger.error("Cannot proceed without esmini installation")
        return

    # Add the project root directory to Python path
    folder_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "test_output"
    )

    logger.info(f"Searching for OpenSCENARIO files in {folder_path}")

    # Statistics for validation results
    total_files = 0
    successful_validations = 0
    failed_validations = 0

    # Get list of all scenario directories directly in test_output
    scenario_dirs = [
        d
        for d in os.listdir(folder_path)
        if os.path.isdir(os.path.join(folder_path, d)) and (d.startswith("road_") or d.startswith("intersection_"))
    ]
    logger.info(f"Found {len(scenario_dirs)} scenarios in {folder_path}")

    # Process all scenarios
    for scenario_dir in scenario_dirs:
        scenario_path = os.path.join(folder_path, scenario_dir)
        
        # Find all OpenSCENARIO files in the scenario directory
        openscenario_files = glob.glob(os.path.join(scenario_path, "**/*.xosc"), recursive=True)
        
        for openscenario_file in openscenario_files:
            total_files += 1
            logger.info(f"Validating file: {openscenario_file}")
            
            # Run esmini for validation
            result = validate_with_esmini(openscenario_file)
            
            if result:
                successful_validations += 1
                logger.info(f"Validation successful: {openscenario_file}")
            else:
                failed_validations += 1
                logger.error(f"Validation failed: {openscenario_file}")
    
    # Print summary
    logger.info("\nValidation Summary:")
    logger.info(f"Total files: {total_files}")
    logger.info(f"Successful validations: {successful_validations}")
    logger.info(f"Failed validations: {failed_validations}")
    logger.info(f"Success rate: {successful_validations / total_files * 100:.2f}% (if any files were processed)")


if __name__ == "__main__":
    main()
