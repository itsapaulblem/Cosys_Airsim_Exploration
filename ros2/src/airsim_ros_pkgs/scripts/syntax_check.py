#!/usr/bin/env python3

import ast
import sys

def check_syntax(filename):
    """Check Python syntax of a file"""
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            source = f.read()
        
        # Try to parse the AST
        ast.parse(source, filename=filename)
        print(f"✓ Syntax check passed for {filename}")
        return True
        
    except SyntaxError as e:
        print(f"✗ Syntax Error in {filename}:")
        print(f"  Line {e.lineno}: {e.text.strip() if e.text else 'N/A'}")
        print(f"  Error: {e.msg}")
        print(f"  Position: {' ' * (e.offset-1) + '^' if e.offset else 'N/A'}")
        return False
        
    except Exception as e:
        print(f"✗ Error checking {filename}: {e}")
        return False

if __name__ == "__main__":
    filename = "motion_detection_node.py"
    success = check_syntax(filename)
    sys.exit(0 if success else 1)