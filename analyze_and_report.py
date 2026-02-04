#!/usr/bin/env python3
"""
Enhanced Navigation Results Analyzer with EDA and Report Generation
Compares NavFn vs SmacPlanner2D global path planners.

Features:
- Parses CSV and JSON result files
- Statistical analysis with mean, std, min, max
- Generates comparison JSON and Markdown reports
- Exploratory Data Analysis (EDA) with interpretations
- Conclusion and recommendations

Usage:
    python3 analyze_and_report.py [results_dir]
"""

import os
import sys
import json
import csv
from datetime import datetime
from collections import defaultdict


def calc_statistics(values):
    """Calculate statistical measures for a list of values."""
    if not values:
        return {'mean': 0, 'std': 0, 'min': 0, 'max': 0, 'count': 0, 'sum': 0}
    
    n = len(values)
    mean = sum(values) / n
    variance = sum((x - mean) ** 2 for x in values) / n if n > 1 else 0
    std = variance ** 0.5
    
    return {
        'mean': round(mean, 3),
        'std': round(std, 3),
        'min': round(min(values), 3),
        'max': round(max(values), 3),
        'count': n,
        'sum': round(sum(values), 3)
    }


def parse_json_file(filepath):
    """Parse a navigation results JSON file."""
    try:
        with open(filepath, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"  Warning: Could not parse {filepath}: {e}")
        return None


def parse_csv_file(filepath):
    """Parse a navigation results CSV file and convert to dict."""
    metrics = {}
    waypoints = []
    
    try:
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
                        'min_obstacle_m': float(row[6]) if row[6] != 'N/A' else None,
                        'success': row[7] == 'True',
                    })
        
        # Convert to JSON-like structure
        return {
            'planner': metrics.get('planner', 'unknown'),
            'run_id': metrics.get('run_id', 'unknown'),
            'timestamp': metrics.get('timestamp', ''),
            'summary': {
                'total_time_sec': float(metrics.get('total_time_sec', 0)),
                'total_distance_m': float(metrics.get('total_distance_m', 0)),
                'total_path_length_m': float(metrics.get('total_path_length_m', 0)),
                'min_obstacle_m': float(metrics.get('min_obstacle_m', 0)) if metrics.get('min_obstacle_m', 'N/A') != 'N/A' else None,
                'success_count': int(metrics.get('success_count', 0)),
                'failure_count': int(metrics.get('failure_count', 0)),
                'waypoints_total': len(waypoints),
                'success_rate_percent': float(metrics.get('success_rate', '0%').replace('%', ''))
            },
            'waypoints': waypoints
        }
    except Exception as e:
        print(f"  Warning: Could not parse CSV {filepath}: {e}")
        return None


def find_results_files(results_dir):
    """Find all result files (CSV and JSON) organized by planner."""
    results = {'navfn': [], 'smac': []}
    
    for root, dirs, files in os.walk(results_dir):
        for file in files:
            filepath = os.path.join(root, file)
            if 'navigation_' in file:
                # Prefer JSON, fallback to CSV
                if file.endswith('.json'):
                    data = parse_json_file(filepath)
                elif file.endswith('.csv'):
                    data = parse_csv_file(filepath)
                else:
                    continue
                
                if data:
                    planner = data.get('planner', '').lower()
                    if 'navfn' in planner or 'navfn' in file.lower():
                        results['navfn'].append(data)
                    elif 'smac' in planner or 'smac' in file.lower():
                        results['smac'].append(data)
    
    return results


def analyze_planner_data(data_list):
    """Analyze all results for a specific planner."""
    if not data_list:
        return None
    
    all_times = []
    all_distances = []
    all_path_lengths = []
    all_min_obstacles = []
    all_efficiencies = []
    success_rates = []
    total_success = 0
    total_waypoints = 0
    
    # Per-waypoint aggregation
    waypoint_times = defaultdict(list)
    waypoint_distances = defaultdict(list)
    
    for data in data_list:
        summary = data.get('summary', {})
        
        all_times.append(summary.get('total_time_sec', 0))
        all_distances.append(summary.get('total_distance_m', 0))
        all_path_lengths.append(summary.get('total_path_length_m', 0))
        
        if summary.get('min_obstacle_m'):
            all_min_obstacles.append(summary['min_obstacle_m'])
        
        if summary.get('path_efficiency_percent'):
            all_efficiencies.append(summary['path_efficiency_percent'])
        elif summary.get('total_path_length_m', 0) > 0:
            eff = summary.get('total_distance_m', 0) / summary['total_path_length_m'] * 100
            all_efficiencies.append(eff)
        
        success_rates.append(summary.get('success_rate_percent', 0))
        total_success += summary.get('success_count', 0)
        total_waypoints += summary.get('waypoints_total', 0)
        
        # Per-waypoint data
        for wp in data.get('waypoints', []):
            name = wp.get('name', '')
            waypoint_times[name].append(wp.get('time_sec', 0))
            waypoint_distances[name].append(wp.get('distance_traveled_m', 0) if 'distance_traveled_m' in wp else wp.get('distance_m', 0))
    
    return {
        'num_runs': len(data_list),
        'total_waypoints': total_waypoints,
        'total_success': total_success,
        'time': calc_statistics(all_times),
        'distance': calc_statistics(all_distances),
        'path_length': calc_statistics(all_path_lengths),
        'obstacle_distance': calc_statistics(all_min_obstacles),
        'efficiency': calc_statistics(all_efficiencies),
        'success_rate': calc_statistics(success_rates),
        'per_waypoint': {
            name: {
                'time': calc_statistics(times),
                'distance': calc_statistics(waypoint_distances[name])
            }
            for name, times in waypoint_times.items()
        }
    }


def generate_comparison_json(navfn_data, smac_data, output_file):
    """Generate JSON comparison report."""
    comparison = {
        'generated': datetime.now().isoformat(),
        'analysis_type': 'Navigation Planner Comparison',
        'planners': {
            'navfn': navfn_data,
            'smac': smac_data
        },
        'comparison': {},
        'conclusions': []
    }
    
    if navfn_data and smac_data:
        # Compare key metrics
        metrics_to_compare = [
            ('time', 'Total Time (s)', 'lower_is_better'),
            ('distance', 'Distance Traveled (m)', 'lower_is_better'),
            ('path_length', 'Path Length (m)', 'lower_is_better'),
            ('obstacle_distance', 'Min Obstacle Distance (m)', 'higher_is_better'),
            ('efficiency', 'Path Efficiency (%)', 'closer_to_100'),
            ('success_rate', 'Success Rate (%)', 'higher_is_better')
        ]
        
        winners = {'navfn': 0, 'smac': 0, 'tie': 0}
        
        for metric_key, metric_name, comparison_type in metrics_to_compare:
            navfn_val = navfn_data[metric_key]['mean'] if navfn_data.get(metric_key) else 0
            smac_val = smac_data[metric_key]['mean'] if smac_data.get(metric_key) else 0
            
            if navfn_val > 0 or smac_val > 0:
                diff_pct = ((smac_val - navfn_val) / navfn_val * 100) if navfn_val > 0 else 0
                
                # Determine winner
                if comparison_type == 'lower_is_better':
                    winner = 'smac' if smac_val < navfn_val else ('navfn' if navfn_val < smac_val else 'tie')
                elif comparison_type == 'higher_is_better':
                    winner = 'smac' if smac_val > navfn_val else ('navfn' if navfn_val > smac_val else 'tie')
                else:  # closer_to_100
                    winner = 'smac' if abs(100 - smac_val) < abs(100 - navfn_val) else 'navfn'
                
                winners[winner] += 1
                
                comparison['comparison'][metric_key] = {
                    'name': metric_name,
                    'navfn': {
                        'mean': navfn_val,
                        'std': navfn_data[metric_key].get('std', 0),
                        'min': navfn_data[metric_key].get('min', 0),
                        'max': navfn_data[metric_key].get('max', 0)
                    },
                    'smac': {
                        'mean': smac_val,
                        'std': smac_data[metric_key].get('std', 0),
                        'min': smac_data[metric_key].get('min', 0),
                        'max': smac_data[metric_key].get('max', 0)
                    },
                    'difference_percent': round(diff_pct, 2),
                    'winner': winner,
                    'interpretation': get_interpretation(metric_key, navfn_val, smac_val, winner)
                }
        
        # Overall winner
        overall_winner = 'smac' if winners['smac'] > winners['navfn'] else ('navfn' if winners['navfn'] > winners['smac'] else 'tie')
        comparison['overall_winner'] = overall_winner
        comparison['wins_summary'] = winners
        
        # Generate conclusions
        comparison['conclusions'] = generate_conclusions(comparison['comparison'], overall_winner, navfn_data, smac_data)
    
    with open(output_file, 'w') as f:
        json.dump(comparison, f, indent=2)
    
    return comparison


def get_interpretation(metric_key, navfn_val, smac_val, winner):
    """Get human-readable interpretation for a metric comparison."""
    diff = abs(smac_val - navfn_val)
    diff_pct = abs((smac_val - navfn_val) / navfn_val * 100) if navfn_val > 0 else 0
    
    interpretations = {
        'time': f"SmacPlanner2D {'completed' if winner == 'smac' else 'took'} {diff:.2f}s {'faster' if winner == 'smac' else 'longer'} ({diff_pct:.1f}% {'improvement' if winner == 'smac' else 'slower'}).",
        'distance': f"Robots traveled {diff:.2f}m {'less' if winner == 'smac' else 'more'} with {'SmacPlanner2D' if winner == 'smac' else 'NavFn'} ({diff_pct:.1f}% {'more efficient' if winner == 'smac' else 'less efficient'}).",
        'path_length': f"{'SmacPlanner2D' if winner == 'smac' else 'NavFn'} generated paths {diff:.2f}m {'shorter' if winner == 'smac' else 'longer'} on average.",
        'obstacle_distance': f"{'SmacPlanner2D' if winner == 'smac' else 'NavFn'} maintained {diff:.2f}m {'greater' if winner == 'smac' else 'smaller'} clearance from obstacles.",
        'efficiency': f"{'SmacPlanner2D' if winner == 'smac' else 'NavFn'} achieved {diff:.1f}% {'better' if winner == 'smac' else 'worse'} path following efficiency.",
        'success_rate': f"{'SmacPlanner2D' if winner == 'smac' else 'NavFn'} had {diff:.1f}% {'higher' if winner == 'smac' else 'lower'} success rate."
    }
    
    return interpretations.get(metric_key, f"{'SmacPlanner2D' if winner == 'smac' else 'NavFn'} performed better.")


def generate_conclusions(comparison_data, overall_winner, navfn_data, smac_data):
    """Generate conclusions based on analysis."""
    conclusions = []
    
    # Overall conclusion
    if overall_winner == 'smac':
        conclusions.append("**Overall Winner: SmacPlanner2D** - The cost-aware A* planner outperformed NavFn in the majority of metrics.")
    elif overall_winner == 'navfn':
        conclusions.append("**Overall Winner: NavFn** - The traditional Dijkstra/A* planner outperformed SmacPlanner2D in the majority of metrics.")
    else:
        conclusions.append("**Result: Tie** - Both planners performed comparably across the tested metrics.")
    
    # Time analysis
    time_comp = comparison_data.get('time', {})
    if time_comp:
        if time_comp['winner'] == 'smac':
            conclusions.append(f"SmacPlanner2D is **{abs(time_comp['difference_percent']):.1f}% faster** on average, completing the delivery route more quickly.")
        elif time_comp['winner'] == 'navfn':
            conclusions.append(f"NavFn is **{abs(time_comp['difference_percent']):.1f}% faster** on average, indicating lower computational overhead.")
    
    # Path quality analysis
    path_comp = comparison_data.get('path_length', {})
    if path_comp:
        if path_comp['winner'] == 'smac':
            conclusions.append("SmacPlanner2D generates **shorter paths**, suggesting better optimization of the global plan.")
        elif path_comp['winner'] == 'navfn':
            conclusions.append("NavFn generates **shorter paths**, indicating efficient wavefront expansion.")
    
    # Safety analysis
    obs_comp = comparison_data.get('obstacle_distance', {})
    if obs_comp:
        if obs_comp['winner'] == 'smac':
            conclusions.append("SmacPlanner2D maintains **larger clearance** from obstacles, enhancing safety in cluttered environments.")
        elif obs_comp['winner'] == 'navfn':
            conclusions.append("NavFn maintains **larger clearance** from obstacles, providing better safety margins.")
    
    # Reliability analysis
    success_comp = comparison_data.get('success_rate', {})
    if success_comp:
        navfn_rate = success_comp.get('navfn', {}).get('mean', 0)
        smac_rate = success_comp.get('smac', {}).get('mean', 0)
        if navfn_rate == 100 and smac_rate == 100:
            conclusions.append("Both planners achieved **100% success rate**, demonstrating high reliability for the test scenarios.")
        elif success_comp['winner'] == 'smac':
            conclusions.append(f"SmacPlanner2D achieved **higher success rate** ({smac_rate:.0f}% vs {navfn_rate:.0f}%), showing better reliability.")
        elif success_comp['winner'] == 'navfn':
            conclusions.append(f"NavFn achieved **higher success rate** ({navfn_rate:.0f}% vs {smac_rate:.0f}%), showing better reliability.")
    
    # Recommendations
    conclusions.append("")
    conclusions.append("### Recommendations")
    if overall_winner == 'smac':
        conclusions.append("- Use **SmacPlanner2D** for complex environments where path optimality and obstacle avoidance are critical.")
        conclusions.append("- SmacPlanner2D's cost-aware search makes it better suited for cluttered indoor spaces.")
    elif overall_winner == 'navfn':
        conclusions.append("- Use **NavFn** for simpler environments or when computational efficiency is prioritized.")
        conclusions.append("- NavFn's simplicity makes it suitable for real-time applications with limited compute.")
    else:
        conclusions.append("- Both planners are viable choices for this environment.")
        conclusions.append("- Consider other factors like computational resources and specific use case requirements.")
    
    return conclusions


def generate_markdown_report(comparison_data, output_file):
    """Generate comprehensive markdown comparison report with EDA."""
    navfn = comparison_data.get('planners', {}).get('navfn', {})
    smac = comparison_data.get('planners', {}).get('smac', {})
    comp = comparison_data.get('comparison', {})
    conclusions = comparison_data.get('conclusions', [])
    
    report = []
    
    # Header
    report.append("# 🤖 Navigation Planner Comparison Report")
    report.append("")
    report.append("## Autonomous Delivery Robot: NavFn vs SmacPlanner2D")
    report.append("")
    report.append(f"**Generated:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append("")
    report.append("---")
    report.append("")
    
    # Executive Summary
    report.append("## 📋 Executive Summary")
    report.append("")
    report.append("This report presents a comprehensive comparison of two Nav2 global path planning algorithms:")
    report.append("")
    report.append("| Algorithm | Description |")
    report.append("|-----------|-------------|")
    report.append("| **NavFn** | Classical grid-based planner using Dijkstra/A* wavefront expansion |")
    report.append("| **SmacPlanner2D** | Modern cost-aware A* planner with optimized search |")
    report.append("")
    
    if navfn and smac:
        report.append(f"**Test runs analyzed:** {navfn.get('num_runs', 0)} NavFn, {smac.get('num_runs', 0)} SmacPlanner2D")
        report.append("")
        overall = comparison_data.get('overall_winner', 'tie').upper()
        report.append(f"**Overall Winner:** {overall}")
        wins = comparison_data.get('wins_summary', {})
        report.append(f"- Metrics won - NavFn: {wins.get('navfn', 0)}, SmacPlanner2D: {wins.get('smac', 0)}, Tie: {wins.get('tie', 0)}")
    report.append("")
    
    # Methodology
    report.append("---")
    report.append("")
    report.append("## 🔬 Methodology")
    report.append("")
    report.append("### Test Environment")
    report.append("- **World:** Custom office environment with rooms, corridors, and furniture")
    report.append("- **Robot:** Differential-drive delivery robot with 2D LiDAR")
    report.append("- **Waypoints:** Reception (2, 1) → Storage (4, 7.5) → Office (12.5, 3)")
    report.append("")
    report.append("### Metrics Collected")
    report.append("| Metric | Description | Unit |")
    report.append("|--------|-------------|------|")
    report.append("| Total Time | Time to complete entire delivery route | seconds |")
    report.append("| Distance Traveled | Actual distance robot moved (from odometry) | meters |")
    report.append("| Path Length | Planned path length (from planner) | meters |")
    report.append("| Min Obstacle Distance | Closest approach to any obstacle | meters |")
    report.append("| Success Rate | Percentage of waypoints reached | % |")
    report.append("| Path Efficiency | Ratio of actual distance to planned path | % |")
    report.append("")
    
    # Detailed Results
    report.append("---")
    report.append("")
    report.append("## 📊 Detailed Results")
    report.append("")
    
    # NavFn Results
    report.append("### NavFn Planner")
    report.append("")
    if navfn:
        report.append(f"**Number of test runs:** {navfn.get('num_runs', 0)}")
        report.append("")
        report.append("| Metric | Mean | Std Dev | Min | Max |")
        report.append("|--------|------|---------|-----|-----|")
        for metric_key in ['time', 'distance', 'path_length', 'obstacle_distance', 'efficiency', 'success_rate']:
            if navfn.get(metric_key):
                m = navfn[metric_key]
                name = metric_key.replace('_', ' ').title()
                report.append(f"| {name} | {m['mean']:.2f} | {m['std']:.2f} | {m['min']:.2f} | {m['max']:.2f} |")
    else:
        report.append("*No NavFn data available*")
    report.append("")
    
    # SmacPlanner2D Results
    report.append("### SmacPlanner2D")
    report.append("")
    if smac:
        report.append(f"**Number of test runs:** {smac.get('num_runs', 0)}")
        report.append("")
        report.append("| Metric | Mean | Std Dev | Min | Max |")
        report.append("|--------|------|---------|-----|-----|")
        for metric_key in ['time', 'distance', 'path_length', 'obstacle_distance', 'efficiency', 'success_rate']:
            if smac.get(metric_key):
                m = smac[metric_key]
                name = metric_key.replace('_', ' ').title()
                report.append(f"| {name} | {m['mean']:.2f} | {m['std']:.2f} | {m['min']:.2f} | {m['max']:.2f} |")
    else:
        report.append("*No SmacPlanner2D data available*")
    report.append("")
    
    # Comparison Table
    report.append("---")
    report.append("")
    report.append("## ⚖️ Head-to-Head Comparison")
    report.append("")
    report.append("| Metric | NavFn | SmacPlanner2D | Δ% | Winner |")
    report.append("|--------|-------|---------------|-----|--------|")
    
    for metric_key in ['time', 'distance', 'path_length', 'obstacle_distance', 'efficiency', 'success_rate']:
        if comp.get(metric_key):
            c = comp[metric_key]
            name = c.get('name', metric_key.replace('_', ' ').title())
            navfn_val = c['navfn']['mean']
            smac_val = c['smac']['mean']
            diff = c['difference_percent']
            winner = c['winner'].upper()
            winner_emoji = '🏆' if winner != 'TIE' else '🤝'
            report.append(f"| {name} | {navfn_val:.2f} | {smac_val:.2f} | {diff:+.1f}% | {winner_emoji} {winner} |")
    report.append("")
    
    # Interpretations
    report.append("---")
    report.append("")
    report.append("## 📈 Analysis & Interpretation")
    report.append("")
    
    for metric_key in ['time', 'distance', 'path_length', 'obstacle_distance', 'success_rate']:
        if comp.get(metric_key):
            c = comp[metric_key]
            report.append(f"### {c.get('name', metric_key.replace('_', ' ').title())}")
            report.append("")
            report.append(c.get('interpretation', ''))
            report.append("")
    
    # EDA Visualizations (text-based)
    report.append("---")
    report.append("")
    report.append("## 📊 EDA Summary")
    report.append("")
    
    if navfn and smac:
        # Time comparison bar
        navfn_time = navfn.get('time', {}).get('mean', 0)
        smac_time = smac.get('time', {}).get('mean', 0)
        max_time = max(navfn_time, smac_time, 1)
        
        report.append("### Time Comparison (seconds)")
        report.append("```")
        report.append(f"NavFn        │{'█' * int(navfn_time/max_time*30):30}│ {navfn_time:.2f}s")
        report.append(f"SmacPlanner  │{'█' * int(smac_time/max_time*30):30}│ {smac_time:.2f}s")
        report.append("```")
        report.append("")
        
        # Distance comparison
        navfn_dist = navfn.get('distance', {}).get('mean', 0)
        smac_dist = smac.get('distance', {}).get('mean', 0)
        max_dist = max(navfn_dist, smac_dist, 1)
        
        report.append("### Distance Traveled (meters)")
        report.append("```")
        report.append(f"NavFn        │{'█' * int(navfn_dist/max_dist*30):30}│ {navfn_dist:.2f}m")
        report.append(f"SmacPlanner  │{'█' * int(smac_dist/max_dist*30):30}│ {smac_dist:.2f}m")
        report.append("```")
        report.append("")
        
        # Success rate
        navfn_success = navfn.get('success_rate', {}).get('mean', 0)
        smac_success = smac.get('success_rate', {}).get('mean', 0)
        
        report.append("### Success Rate (%)")
        report.append("```")
        report.append(f"NavFn        │{'█' * int(navfn_success/100*30):30}│ {navfn_success:.0f}%")
        report.append(f"SmacPlanner  │{'█' * int(smac_success/100*30):30}│ {smac_success:.0f}%")
        report.append("```")
    report.append("")
    
    # Conclusions
    report.append("---")
    report.append("")
    report.append("## 🎯 Conclusions")
    report.append("")
    for conclusion in conclusions:
        if conclusion.startswith("###"):
            report.append(conclusion)
        elif conclusion.startswith("-"):
            report.append(conclusion)
        elif conclusion:
            report.append(conclusion)
        else:
            report.append("")
    report.append("")
    
    # Footer
    report.append("---")
    report.append("")
    report.append("## 📝 Notes")
    report.append("")
    report.append("- Results may vary based on simulation conditions and hardware performance")
    report.append("- For production use, consider running additional tests with varied environments")
    report.append("- Both planners use the same local controller (DWB) for trajectory tracking")
    report.append("")
    report.append("---")
    report.append("")
    report.append("*Report generated automatically by Navigation Analysis Tool*")
    report.append(f"*Analysis timestamp: {datetime.now().isoformat()}*")
    
    with open(output_file, 'w') as f:
        f.write('\n'.join(report))
    
    return report


def print_console_summary(comparison_data):
    """Print a summary to the console."""
    print("\n" + "=" * 70)
    print("  NAVIGATION PLANNER COMPARISON SUMMARY")
    print("=" * 70)
    
    navfn = comparison_data.get('planners', {}).get('navfn', {})
    smac = comparison_data.get('planners', {}).get('smac', {})
    comp = comparison_data.get('comparison', {})
    
    if navfn:
        print(f"\n📊 NavFn: {navfn.get('num_runs', 0)} runs")
        print(f"   Time: {navfn.get('time', {}).get('mean', 0):.2f}s ± {navfn.get('time', {}).get('std', 0):.2f}s")
        print(f"   Distance: {navfn.get('distance', {}).get('mean', 0):.2f}m")
        print(f"   Success: {navfn.get('success_rate', {}).get('mean', 0):.0f}%")
    
    if smac:
        print(f"\n📊 SmacPlanner2D: {smac.get('num_runs', 0)} runs")
        print(f"   Time: {smac.get('time', {}).get('mean', 0):.2f}s ± {smac.get('time', {}).get('std', 0):.2f}s")
        print(f"   Distance: {smac.get('distance', {}).get('mean', 0):.2f}m")
        print(f"   Success: {smac.get('success_rate', {}).get('mean', 0):.0f}%")
    
    print("\n" + "-" * 70)
    print("  WINNER: " + comparison_data.get('overall_winner', 'TIE').upper())
    wins = comparison_data.get('wins_summary', {})
    print(f"  Metrics: NavFn={wins.get('navfn', 0)}, SmacPlanner2D={wins.get('smac', 0)}, Tie={wins.get('tie', 0)}")
    print("=" * 70 + "\n")


def main():
    results_dir = sys.argv[1] if len(sys.argv) > 1 else 'test_results'
    
    if not os.path.exists(results_dir):
        print(f"❌ Results directory not found: {results_dir}")
        print("   Run navigation tests first to generate data.")
        sys.exit(1)
    
    print(f"🔍 Analyzing results in: {results_dir}")
    
    # Find and parse result files
    result_data = find_results_files(results_dir)
    print(f"   Found {len(result_data['navfn'])} NavFn results")
    print(f"   Found {len(result_data['smac'])} SmacPlanner2D results")
    
    if not result_data['navfn'] and not result_data['smac']:
        print("❌ No navigation result files found!")
        print("   Expected files matching: navigation_*.json or navigation_*.csv")
        sys.exit(1)
    
    # Analyze each planner
    print("\n📈 Analyzing NavFn results...")
    navfn_analysis = analyze_planner_data(result_data['navfn'])
    
    print("📈 Analyzing SmacPlanner2D results...")
    smac_analysis = analyze_planner_data(result_data['smac'])
    
    # Generate outputs
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    
    json_report = os.path.join(results_dir, f'comparison_report_{timestamp}.json')
    md_report = os.path.join(results_dir, f'comparison_report_{timestamp}.md')
    
    print("\n📝 Generating comparison reports...")
    comparison = generate_comparison_json(navfn_analysis, smac_analysis, json_report)
    generate_markdown_report(comparison, md_report)
    
    # Print console summary
    print_console_summary(comparison)
    
    print(f"✅ JSON report saved: {json_report}")
    print(f"✅ Markdown report saved: {md_report}")
    print(f"\n📄 View the full report:")
    print(f"   cat {md_report}")


if __name__ == '__main__':
    main()

