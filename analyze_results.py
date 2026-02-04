#!/usr/bin/env python3
"""
Navigation Results Analyzer
Compares NavFn vs SmacPlanner2D performance metrics.

Usage:
    python3 analyze_results.py [results_dir]
    
If no directory specified, looks for test_results/ in current directory.
"""

import os
import sys
import csv
from datetime import datetime
from collections import defaultdict


def parse_csv_file(filepath):
    """Parse a navigation results CSV file."""
    metrics = {}
    waypoints = []
    
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        section = None
        
        for row in reader:
            if not row:
                continue
            
            if row[0].startswith('==='):
                if 'WAYPOINT' in row[0]:
                    section = 'waypoints'
                else:
                    section = 'summary'
                continue
            
            if section == 'summary' and len(row) >= 2:
                key, value = row[0], row[1]
                metrics[key] = value
            elif section == 'waypoints' and len(row) >= 8:
                if row[0] == 'waypoint':  # Header row
                    continue
                waypoints.append({
                    'name': row[0],
                    'goal_x': float(row[1]),
                    'goal_y': float(row[2]),
                    'time_sec': float(row[3]),
                    'distance_m': float(row[4]),
                    'path_length_m': float(row[5]),
                    'min_obstacle_m': row[6] if row[6] != 'N/A' else None,
                    'success': row[7] == 'True',
                })
    
    metrics['waypoints'] = waypoints
    return metrics


def find_results_files(results_dir):
    """Find all CSV result files organized by planner."""
    results = {'navfn': [], 'smac': []}
    
    for root, dirs, files in os.walk(results_dir):
        for file in files:
            if file.endswith('.csv') and 'navigation_' in file:
                filepath = os.path.join(root, file)
                if 'navfn' in filepath.lower() or 'navfn' in file.lower():
                    results['navfn'].append(filepath)
                elif 'smac' in filepath.lower() or 'smac' in file.lower():
                    results['smac'].append(filepath)
    
    return results


def calculate_statistics(values):
    """Calculate basic statistics for a list of values."""
    if not values:
        return {'min': 0, 'max': 0, 'mean': 0, 'count': 0}
    
    return {
        'min': min(values),
        'max': max(values),
        'mean': sum(values) / len(values),
        'count': len(values),
    }


def analyze_planner_results(result_files):
    """Analyze all results for a specific planner."""
    if not result_files:
        return None
    
    all_times = []
    all_distances = []
    all_path_lengths = []
    all_min_obstacles = []
    success_count = 0
    total_waypoints = 0
    
    for filepath in result_files:
        try:
            metrics = parse_csv_file(filepath)
            
            # Extract numeric values
            if 'total_time_sec' in metrics:
                all_times.append(float(metrics['total_time_sec']))
            if 'total_distance_m' in metrics:
                all_distances.append(float(metrics['total_distance_m']))
            if 'total_path_length_m' in metrics:
                all_path_lengths.append(float(metrics['total_path_length_m']))
            if 'min_obstacle_m' in metrics and metrics['min_obstacle_m'] != 'N/A':
                all_min_obstacles.append(float(metrics['min_obstacle_m']))
            if 'success_count' in metrics:
                success_count += int(metrics['success_count'])
            
            # Count waypoints
            total_waypoints += len(metrics.get('waypoints', []))
            
        except Exception as e:
            print(f"  Warning: Could not parse {filepath}: {e}")
    
    return {
        'num_runs': len(result_files),
        'time_stats': calculate_statistics(all_times),
        'distance_stats': calculate_statistics(all_distances),
        'path_length_stats': calculate_statistics(all_path_lengths),
        'obstacle_stats': calculate_statistics(all_min_obstacles),
        'success_count': success_count,
        'total_waypoints': total_waypoints,
    }


def generate_report(navfn_analysis, smac_analysis, output_file):
    """Generate a comparison report."""
    
    report = []
    report.append("=" * 80)
    report.append("       NAVIGATION PLANNER COMPARISON REPORT")
    report.append("=" * 80)
    report.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append("")
    
    # Summary table header
    report.append("-" * 80)
    report.append(f"{'Metric':<30} {'NavFn':<20} {'SmacPlanner2D':<20} {'Winner':<10}")
    report.append("-" * 80)
    
    # Compare metrics
    def compare_metric(name, navfn_val, smac_val, lower_is_better=True):
        if navfn_val is None and smac_val is None:
            return f"{name:<30} {'N/A':<20} {'N/A':<20} {'-':<10}"
        
        navfn_str = f"{navfn_val:.2f}" if navfn_val is not None else "N/A"
        smac_str = f"{smac_val:.2f}" if smac_val is not None else "N/A"
        
        if navfn_val is None:
            winner = "SMAC"
        elif smac_val is None:
            winner = "NavFn"
        elif lower_is_better:
            winner = "NavFn" if navfn_val < smac_val else ("SMAC" if smac_val < navfn_val else "TIE")
        else:
            winner = "NavFn" if navfn_val > smac_val else ("SMAC" if smac_val > navfn_val else "TIE")
        
        return f"{name:<30} {navfn_str:<20} {smac_str:<20} {winner:<10}"
    
    # Extract values safely
    def get_stat(analysis, stat_name, key='mean'):
        if analysis and stat_name in analysis:
            return analysis[stat_name].get(key)
        return None
    
    # Add metrics to report
    report.append(compare_metric(
        "Avg Time (sec)",
        get_stat(navfn_analysis, 'time_stats'),
        get_stat(smac_analysis, 'time_stats'),
        lower_is_better=True
    ))
    
    report.append(compare_metric(
        "Avg Distance Traveled (m)",
        get_stat(navfn_analysis, 'distance_stats'),
        get_stat(smac_analysis, 'distance_stats'),
        lower_is_better=True
    ))
    
    report.append(compare_metric(
        "Avg Path Length (m)",
        get_stat(navfn_analysis, 'path_length_stats'),
        get_stat(smac_analysis, 'path_length_stats'),
        lower_is_better=True
    ))
    
    report.append(compare_metric(
        "Min Obstacle Distance (m)",
        get_stat(navfn_analysis, 'obstacle_stats'),
        get_stat(smac_analysis, 'obstacle_stats'),
        lower_is_better=False  # Higher is safer
    ))
    
    # Success rates
    navfn_success = None
    smac_success = None
    if navfn_analysis and navfn_analysis['total_waypoints'] > 0:
        navfn_success = navfn_analysis['success_count'] / navfn_analysis['total_waypoints'] * 100
    if smac_analysis and smac_analysis['total_waypoints'] > 0:
        smac_success = smac_analysis['success_count'] / smac_analysis['total_waypoints'] * 100
    
    report.append(compare_metric(
        "Success Rate (%)",
        navfn_success,
        smac_success,
        lower_is_better=False
    ))
    
    report.append("-" * 80)
    
    # Detailed statistics
    report.append("")
    report.append("=" * 80)
    report.append("                    DETAILED STATISTICS")
    report.append("=" * 80)
    
    for planner_name, analysis in [("NavFn", navfn_analysis), ("SmacPlanner2D", smac_analysis)]:
        report.append("")
        report.append(f"--- {planner_name} ---")
        if analysis:
            report.append(f"  Number of runs: {analysis['num_runs']}")
            report.append(f"  Total waypoints attempted: {analysis['total_waypoints']}")
            report.append(f"  Successful waypoints: {analysis['success_count']}")
            
            for stat_name, stat_label in [
                ('time_stats', 'Time (sec)'),
                ('distance_stats', 'Distance (m)'),
                ('path_length_stats', 'Path Length (m)'),
                ('obstacle_stats', 'Min Obstacle (m)')
            ]:
                stats = analysis.get(stat_name, {})
                if stats.get('count', 0) > 0:
                    report.append(f"  {stat_label}:")
                    report.append(f"    Min: {stats['min']:.2f}, Max: {stats['max']:.2f}, Mean: {stats['mean']:.2f}")
        else:
            report.append("  No results available")
    
    # Conclusions
    report.append("")
    report.append("=" * 80)
    report.append("                       CONCLUSIONS")
    report.append("=" * 80)
    
    if navfn_analysis and smac_analysis:
        navfn_time = get_stat(navfn_analysis, 'time_stats')
        smac_time = get_stat(smac_analysis, 'time_stats')
        navfn_dist = get_stat(navfn_analysis, 'distance_stats')
        smac_dist = get_stat(smac_analysis, 'distance_stats')
        
        if navfn_time and smac_time:
            if navfn_time < smac_time:
                report.append(f"• NavFn is {((smac_time - navfn_time) / smac_time * 100):.1f}% faster on average")
            elif smac_time < navfn_time:
                report.append(f"• SmacPlanner2D is {((navfn_time - smac_time) / navfn_time * 100):.1f}% faster on average")
        
        if navfn_dist and smac_dist:
            if navfn_dist < smac_dist:
                report.append(f"• NavFn paths are {((smac_dist - navfn_dist) / smac_dist * 100):.1f}% shorter on average")
            elif smac_dist < navfn_dist:
                report.append(f"• SmacPlanner2D paths are {((navfn_dist - smac_dist) / navfn_dist * 100):.1f}% shorter on average")
        
        if navfn_success and smac_success:
            if navfn_success > smac_success:
                report.append(f"• NavFn has higher success rate ({navfn_success:.1f}% vs {smac_success:.1f}%)")
            elif smac_success > navfn_success:
                report.append(f"• SmacPlanner2D has higher success rate ({smac_success:.1f}% vs {navfn_success:.1f}%)")
            else:
                report.append(f"• Both planners have equal success rate ({navfn_success:.1f}%)")
    else:
        report.append("Insufficient data for comparison conclusions.")
    
    report.append("")
    report.append("=" * 80)
    report.append("                      END OF REPORT")
    report.append("=" * 80)
    
    # Write report
    report_text = "\n".join(report)
    
    with open(output_file, 'w') as f:
        f.write(report_text)
    
    return report_text


def main():
    # Determine results directory
    if len(sys.argv) > 1:
        results_dir = sys.argv[1]
    else:
        results_dir = os.path.join(os.path.dirname(__file__), 'test_results')
    
    if not os.path.exists(results_dir):
        print(f"Results directory not found: {results_dir}")
        print("Run navigation tests first to generate results.")
        sys.exit(1)
    
    print(f"Analyzing results in: {results_dir}")
    
    # Find result files
    result_files = find_results_files(results_dir)
    print(f"  Found {len(result_files['navfn'])} NavFn result files")
    print(f"  Found {len(result_files['smac'])} SmacPlanner2D result files")
    
    if not result_files['navfn'] and not result_files['smac']:
        print("No result files found!")
        sys.exit(1)
    
    # Analyze results
    print("\nAnalyzing NavFn results...")
    navfn_analysis = analyze_planner_results(result_files['navfn'])
    
    print("Analyzing SmacPlanner2D results...")
    smac_analysis = analyze_planner_results(result_files['smac'])
    
    # Generate report
    report_file = os.path.join(results_dir, f'comparison_report_{datetime.now().strftime("%Y%m%d_%H%M%S")}.txt')
    report_text = generate_report(navfn_analysis, smac_analysis, report_file)
    
    # Print report
    print("\n" + report_text)
    print(f"\nReport saved to: {report_file}")


if __name__ == '__main__':
    main()

