#!/bin/bash

# Data Science Project Cleanup Script
# Removes temporary files, cache, and system files

echo "🧹 Starting Data Science Project Cleanup..."
echo "=========================================="

# Remove macOS system files
echo "Removing .DS_Store files..."
find . -name ".DS_Store" -type f -delete
echo "✅ .DS_Store files removed"

# Remove Python cache directories
echo "Removing Python cache directories..."
find . -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
echo "✅ Python cache directories removed"

# Remove Python compiled files
echo "Removing Python compiled files..."
find . -name "*.pyc" -type f -delete 2>/dev/null || true
find . -name "*.pyo" -type f -delete 2>/dev/null || true
echo "✅ Python compiled files removed"

# Remove temporary files
echo "Removing temporary files..."
find . -name "*.tmp" -type f -delete 2>/dev/null || true
find . -name "*.temp" -type f -delete 2>/dev/null || true
find . -name "*.log" -type f -delete 2>/dev/null || true
find . -name "*.out" -type f -delete 2>/dev/null || true
echo "✅ Temporary files removed"

# Remove Jupyter checkpoint directories
echo "Removing Jupyter checkpoint directories..."
find . -name ".ipynb_checkpoints" -type d -exec rm -rf {} + 2>/dev/null || true
echo "✅ Jupyter checkpoints removed"

# Remove editor temporary files
echo "Removing editor temporary files..."
find . -name "*~" -type f -delete 2>/dev/null || true
find . -name "*.swp" -type f -delete 2>/dev/null || true
find . -name "*.swo" -type f -delete 2>/dev/null || true
echo "✅ Editor temporary files removed"

# Show current directory size
echo ""
echo "📊 Cleanup Summary:"
echo "==================="
du -sh . 2>/dev/null | awk '{print "Total directory size: " $1}'

# Count files by type
echo ""
echo "📁 File Count by Type:"
echo "====================="
echo "Python files: $(find . -name "*.py" -type f | wc -l | tr -d ' ')"
echo "Jupyter notebooks: $(find . -name "*.ipynb" -type f | wc -l | tr -d ' ')"
echo "Data files (CSV): $(find . -name "*.csv" -type f | wc -l | tr -d ' ')"
echo "Model files (H5): $(find . -name "*.h5" -type f | wc -l | tr -d ' ')"
echo "Image files (PNG): $(find . -name "*.png" -type f | wc -l | tr -d ' ')"
echo "Documentation (MD): $(find . -name "*.md" -type f | wc -l | tr -d ' ')"

echo ""
echo "✅ Cleanup completed successfully!"
echo "🚀 Your Data Science projects are now clean and organized."
