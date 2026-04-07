#!/usr/bin/env python3
"""Compare determinism test hashes across platforms."""

import os
import sys
import re
from pathlib import Path

def extract_hash(content: str) -> str:
    """Extract the final hash from test output."""
    match = re.search(r'Final hash:\s+([0-9a-fA-F]+)', content)
    if not match:
        return None
    return match.group(1).lower()

def extract_checkpoints(content: str) -> list:
    """Extract all checkpoint hashes for mid-sim divergence detection."""
    return re.findall(r'\[.*?\] hash=([0-9a-fA-F]+)', content)

def main():
    results_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("results")
    
    platform_hashes = {}
    platform_checkpoints = {}
    
    # Read all result files
    for artifact_dir in results_dir.iterdir():
        if not artifact_dir.is_dir():
            continue
        for result_file in artifact_dir.glob("det_output_*.txt"):
            platform_name = artifact_dir.name.replace("det-results-", "")
            content = result_file.read_text()
            
            final_hash = extract_hash(content)
            if final_hash:
                platform_hashes[platform_name] = final_hash
            
            checkpoints = extract_checkpoints(content)
            if checkpoints:
                platform_checkpoints[platform_name] = checkpoints
    
    print("=" * 60)
    print("APC DETERMINISM CROSS-PLATFORM COMPARISON")
    print("=" * 60)
    
    # Check for mismatches
    failed = False
    reference_hash = None
    reference_platform = None
    
    for platform, hash_val in sorted(platform_hashes.items()):
        if reference_hash is None:
            reference_hash = hash_val
            reference_platform = platform
            print(f"  {platform:20s}: {hash_val}  (REFERENCE)")
        elif hash_val != reference_hash:
            print(f"  {platform:20s}: {hash_val}  *** MISMATCH ***")
            failed = True
        else:
            print(f"  {platform:20s}: {hash_val}  (OK)")
    
    print()
    
    # Check checkpoint divergence (finds WHERE sim diverges)
    if failed:
        print("MID-SIMULATION DIVERGENCE ANALYSIS:")
        all_checkpoints = list(platform_checkpoints.values())
        if len(all_checkpoints) >= 2:
            ref = all_checkpoints[0]
            for i, cp in enumerate(ref):
                for j, other_cps in enumerate(all_checkpoints[1:], 1):
                    platforms = list(platform_checkpoints.keys())
                    if i < len(other_cps) and cp != other_cps[i]:
                        print(f"  First divergence at checkpoint {i} (step {i*100})")
                        print(f"    {platforms[0]}: {cp}")
                        print(f"    {platforms[j]}: {other_cps[i]}")
                        # Don't break - find ALL divergences
        print()
    
    # Summary
    if failed:
        print("RESULT: *** DETERMINISM FAILURE ***")
        print("Action required: Investigate FP behavior differences")
        with open(results_dir / "DETERMINISM_FAILED", "w") as f:
            f.write("1")
        sys.exit(1)
    else:
        print(f"RESULT: ALL {len(platform_hashes)} PLATFORMS MATCH")
        print("Determinism verified.")
        sys.exit(0)

if __name__ == "__main__":
    main()