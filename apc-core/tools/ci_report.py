#!/usr/bin/env python3
"""Generates a GitHub Actions summary for the APC Determinism CI pipeline."""

import os
import sys
import re
from pathlib import Path

def extract_metadata(content: str) -> dict:
    """Extract hash, platform, and compiler from determinism output."""
    hash_match = re.search(r'Final hash:\s+([0-9a-fA-F]+)', content)
    platform_match = re.search(r'Platform:\s+(.+)', content)
    compiler_match = re.search(r'Compiler:\s+(.+)', content)

    return {
        'hash': hash_match.group(1) if hash_match else "N/A",
        'platform': platform_match.group(1).strip() if platform_match else "Unknown",
        'compiler': compiler_match.group(1).strip() if compiler_match else "Unknown",
    }

def main():
    # Allow overriding the directory via command line, default to 'results'
    results_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("results")
    
    if not results_dir.exists():
        print(f"Error: Results directory '{results_dir}' not found.")
        sys.exit(1)

    # Read all result files downloaded by the CI artifact step
    platforms = {}
    for artifact_dir in results_dir.iterdir():
        if not artifact_dir.is_dir():
            continue
            
        for result_file in artifact_dir.glob("det_output_*.txt"):
            # Extract a clean name from the folder (e.g., 'det-results-linux-gcc' -> 'linux-gcc')
            platform_name = artifact_dir.name.replace("det-results-", "")
            content = result_file.read_text()
            platforms[platform_name] = extract_metadata(content)

    if not platforms:
        print("Error: No det_output_*.txt files found in results directory.")
        sys.exit(1)

    # Generate Markdown Summary
    summary_lines = []
    summary_lines.append("## 🛡️ APC Determinism Verification Report\n")
    summary_lines.append("| Platform | Compiler | Final Hash | Status |")
    summary_lines.append("|----------|----------|------------|--------|")

    reference_hash = None
    has_failure = False

    for name, meta in sorted(platforms.items()):
        if reference_hash is None:
            # First platform becomes the reference
            reference_hash = meta['hash']
            status = "✅ **REFERENCE**"
        elif meta['hash'] != reference_hash:
            status = "❌ **MISMATCH**"
            has_failure = True
        else:
            status = "✅ **MATCH**"

        summary_lines.append(f"| {meta['platform']} (`{name}`) | {meta['compiler']} | `{meta['hash']}` | {status} |")

    # Footer summary
    if has_failure:
        summary_lines.append("\n### ⚠️ Determinism Failure Detected")
        summary_lines.append("Cross-platform hashes do not match. Investigate floating-point behavior, compiler flags, or platform-specific math libraries.")
    else:
        summary_lines.append(f"\n### ✅ Success")
        summary_lines.append(f"All {len(platforms)} platforms match the reference baseline.")

    markdown_output = "\n".join(summary_lines)
    
    # 1. Write to GitHub Actions summary file if the env var is set
    github_summary = os.getenv("GITHUB_STEP_SUMMARY")
    if github_summary:
        with open(github_summary, "a") as f:
            f.write("\n" + markdown_output + "\n")
            
    # 2. Always print to stdout for local debugging / CI logs
    print(markdown_output)

    # Exit with failure code if hashes mismatched (stops CI pipeline)
    if has_failure:
        sys.exit(1)

if __name__ == "__main__":
    main()