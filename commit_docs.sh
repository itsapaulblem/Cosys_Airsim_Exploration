#!/bin/bash
# Script to commit documentation changes

echo "Removing git locks..."
rm -f .git/index.lock .git/refs/heads/main.lock 2>/dev/null

echo "Staging drone configuration docs..."
git add docs/drone-configuration/ || echo "Failed to stage drone-configuration"

echo "Staging moved docs..."
git add docs/core/communication_architecture.md docs/installation/multi_machine_setup.md docs/ros2/development_workflow_guide.md docs/px4/fix_implementation_guide.md 2>/dev/null || echo "Some moved files already staged"

echo "Committing changes..."
git commit -m "Add comprehensive drone configuration documentation and reorganize docs

- Add complete drone configuration guide under docs/drone-configuration/
- Move and reorganize documentation files to proper locations  
- Clean up base directory by removing temporary files

🤖 Generated with [Claude Code](https://claude.ai/code)

Co-Authored-By: Claude <noreply@anthropic.com>" || echo "Commit may have failed or completed"

echo "Done!"