#!/usr/bin/env python3
"""
Script to remove all emojis from Python files in the mission_planning directory.
This fixes Windows console encoding issues with emoji characters.
"""

import os
import re
import glob

def remove_emojis_from_text(text):
    """Remove all emoji characters from text."""
    # Comprehensive Unicode ranges for emojis and special characters
    emoji_pattern = re.compile(
        "["
        "\U0001F600-\U0001F64F"  # emoticons
        "\U0001F300-\U0001F5FF"  # symbols & pictographs
        "\U0001F680-\U0001F6FF"  # transport & map symbols
        "\U0001F1E0-\U0001F1FF"  # flags (iOS)
        "\U00002700-\U000027BF"  # dingbats
        "\U0001F900-\U0001F9FF"  # supplemental symbols
        "\U00002600-\U000026FF"  # miscellaneous symbols
        "\U00002B00-\U00002BFF"  # miscellaneous symbols and arrows
        "\U0001F004-\U0001F0CF"  # playing cards, etc.
        "\U0001F170-\U0001F251"  # enclosed characters
        "\U00002328\U000023CF"   # keyboard, eject symbol
        "\U000023E9-\U000023FA"  # play/pause buttons
        "\U000025AA-\U000025AB"  # geometric shapes
        "\U000025B6\U000025C0"   # play buttons
        "\U000025FB-\U000025FE"  # squares
        "\U00002B05-\U00002B07"  # arrows
        "\U00002B1B-\U00002B1C"  # squares
        "\U00002B50\U00002B55"   # star, circle
        "\U0001F191-\U0001F251"  # squared letters
        # Variation selectors that cause encoding issues
        "\uFE0E\uFE0F"           # variation selectors 
        "]+", 
        flags=re.UNICODE
    )
    
    # Also remove specific problematic emoji sequences found in files
    specific_emojis = [
        "⚠️", "❌", "✅", "🔄", "🎯", "🚨", "🔊"
    ]
    
    cleaned_text = emoji_pattern.sub('', text)
    
    # Remove specific emojis one by one
    for emoji in specific_emojis:
        cleaned_text = cleaned_text.replace(emoji, "")
    
    return cleaned_text

def process_file(file_path):
    """Remove emojis from a single file."""
    print(f"Processing: {file_path}")
    
    # Read the file
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
    except UnicodeDecodeError:
        print(f"  Warning: Could not read {file_path} with UTF-8, trying with latin-1")
        with open(file_path, 'r', encoding='latin-1') as f:
            content = f.read()
    
    # Remove emojis
    original_length = len(content)
    cleaned_content = remove_emojis_from_text(content)
    
    # Check if any changes were made
    if len(cleaned_content) != original_length:
        print(f"  Removed {original_length - len(cleaned_content)} emoji characters")
        
        # Write back the cleaned content
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(cleaned_content)
        print(f"  Updated: {file_path}")
    else:
        print(f"  No emojis found in: {file_path}")

def main():
    """Main function to process all Python files in mission_planning directory and other key files."""
    print("Comprehensive Emoji Remover for Mission Planning Files")
    print("=" * 60)
    
    # Process mission_planning directory
    mission_planning_dir = "PythonClient/multirotor/mission_planning"
    if os.path.exists(mission_planning_dir):
        python_files = glob.glob(os.path.join(mission_planning_dir, "*.py"))
        print(f"Found {len(python_files)} Python files in mission_planning:")
        for file_path in python_files:
            process_file(file_path)
    
    # Process multirotor directory (for any mission files there)
    multirotor_dir = "PythonClient/multirotor"
    if os.path.exists(multirotor_dir):
        mission_files = glob.glob(os.path.join(multirotor_dir, "*mission*.py"))
        if mission_files:
            print(f"\nFound {len(mission_files)} mission files in multirotor directory:")
            for file_path in mission_files:
                process_file(file_path)
    
    # Process Unreal directories (where mission files might be copied)
    unreal_python_dirs = [
        "Unreal/Environments/BlankV1/Content/Python",
        "Unreal/Environments/Blocks/Content/Python",
        "Unreal/Environments/AirSimNH/Content/Python"
    ]
    
    for unreal_dir in unreal_python_dirs:
        if os.path.exists(unreal_dir):
            python_files = glob.glob(os.path.join(unreal_dir, "*.py"))
            if python_files:
                print(f"\nFound {len(python_files)} Python files in {unreal_dir}:")
                for file_path in python_files:
                    process_file(file_path)
    
    # Process other key Python files in PythonClient directory
    other_files = [
        "PythonClient/mission_api_server.py",
        "PythonClient/streamlit_mission_control.py",
        "PythonClient/test_airsim_connection.py",
        "PythonClient/level_manager.py",
        "PythonClient/demo_streamlit.py",
        "PythonClient/setup_airsim_env.py",
        "PythonClient/fix_dependency_conflicts.py",
        "PythonClient/quick_setup.py"
    ]
    
    print(f"\nProcessing other key files:")
    for file_path in other_files:
        if os.path.exists(file_path):
            process_file(file_path)
        else:
            print(f"File not found (skipping): {file_path}")
    
    # Also process root directory test files that might contain emojis
    root_test_files = glob.glob("test_*.py")
    if root_test_files:
        print(f"\nProcessing root test files:")
        for file_path in root_test_files:
            process_file(file_path)
    
    print("\n" + "=" * 60)
    print("Comprehensive emoji removal complete!")
    print("All Python files have been updated to remove emoji characters for Windows console compatibility.")
    print("This includes mission files in PythonClient, Unreal directories, and test files.")

if __name__ == "__main__":
    main() 