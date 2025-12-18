#!/usr/bin/env python3
"""
ASP Minimal Example - Step 1: Hello World

This is the simplest possible example demonstrating how to use program.lp.
It shows:
1. How to set up Clingo with constants
2. How to provide input via external facts
3. How to extract output from answer sets

No visualization - just prints the solution.
"""

import clingo
from clingo import Number, Function

# Action names for better readability
ACTION_NAMES = {0: "UP", 1: "DOWN", 2: "LEFT", 3: "RIGHT"}


def on_model(model):
    
    """Callback function called when ASP finds a solution"""
    print("\n" + "=" * 70)
    print("OUTPUT: ASP Solution (Answer Set)")
    print("=" * 70)
    print("Raw result:")
    print(model)
    print("-" * 70)
    
    actions = []
    action_details = []
    
    for atom in model.symbols(shown=True):
        if atom.name == "action_taken":
            action = atom.arguments[0].number
            time_step = atom.arguments[1].number
            col = atom.arguments[2].number
            row = atom.arguments[3].number
            
            # Ensure list is big enough
            while len(actions) <= time_step:
                actions.append(None)
            actions[time_step] = action
            
            action_details.append({
                'action': action,
                'time': time_step,
                'col': col,
                'row': row
            })
    
    # Print all action_taken atoms
    print("\nAnswer Set Atoms:")
    print("-" * 70)
    for detail in sorted(action_details, key=lambda x: x['time']):
        action_name = ACTION_NAMES.get(detail['action'], f"UNKNOWN({detail['action']})")
        print(f"  action_taken({detail['action']}, {detail['time']}, {detail['col']}, {detail['row']})")
        print(f"    → Action {detail['action']} ({action_name}) at time step {detail['time']}")
        print(f"    → Taken from position ({detail['col']}, {detail['row']})")
    
    # Print summary
    print("\n" + "-" * 70)
    print("Solution Summary:")
    print("-" * 70)
    for t, action in enumerate(actions):
        if action is not None:
            action_name = ACTION_NAMES.get(action, f"UNKNOWN({action})")
            print(f"  Time step {t}: Action {action} ({action_name})")
    
    print("=" * 70 + "\n")
    return actions


def hello_world_example():
    """Simplest possible example"""
    
    print("=" * 70)
    print("ASP Navigation - Hello World Example")
    print("=" * 70)
    
    # Step 1: Create Clingo control with minimal constants
    print("\n[Step 1] Creating Clingo control with constants...")
    constants = {
        "horizon": 3,
        "obstacles": 0,
        "radius": 3,
        "size": 10
    }
    ctl = clingo.Control([
        "-c", f"horizon={constants['horizon']}",
        "-c", f"obstacles={constants['obstacles']}",
        "-c", f"radius={constants['radius']}",
        "-c", f"size={constants['size']}"
    ])
    
    # Step 2: Load the ASP program
    print("[Step 2] Loading program.lp...")
    ctl.load('program.lp')
    
    # Step 3: Ground the program
    print("[Step 3] Grounding the program...")
    ctl.ground([("base", [])])
    print("✓ Program loaded and grounded")
    
    # Step 4: Set external facts (the "input")
    print("\n" + "=" * 70)
    print("INPUT: External Facts (What we provide to ASP)")
    print("=" * 70)
    
    # Store inputs for display
    inputs = {
        'constants': constants,
        'targets': [],
        'walls': [],
        'restricted_areas': [],
        'obstacles': [],
        'multi': None,
        'actions': []
    }
    
    # Robot starts at center (0,0) - this is implicit in program.lp
    print("\n1. Robot Position:")
    print("   - Robot at center (0, 0) - implicit in program.lp")
    print("   - (Robot is always at relative position (0,0) to itself)")
    
    # Target is 2 cells to the right
    target_pos = (2, 0)
    ctl.assign_external(Function("target", [Number(target_pos[0]), Number(target_pos[1])]), True)
    inputs['targets'].append(target_pos)
    print("\n2. Target:")
    print(f"   - target({target_pos[0]}, {target_pos[1]}) = TRUE")
    print(f"   - Target at relative position ({target_pos[0]}, {target_pos[1]})")
    
    # No walls, no obstacles, no restricted areas for simplicity
    print("\n3. Walls:")
    print("   - None (all wall(C,R) = FALSE)")
    
    print("\n4. Restricted Areas:")
    print("   - None (all restricted_area(C,R) = FALSE)")
    
    print("\n5. Obstacles:")
    print("   - None (obstacles = 0)")
    
    # Set visited multiplier (1 = first visit)
    multi_value = 1
    ctl.assign_external(Function("multi", [Number(multi_value)]), True)
    inputs['multi'] = multi_value
    print("\n6. Visited Multiplier:")
    print(f"   - multi({multi_value}) = TRUE")
    print(f"   - Robot has visited current cell {multi_value} time(s)")
    
    # Set RL preferences: prefer moving right (action 3) at all positions
    print("\n7. RL Action Preferences:")
    print("   - For each cell in window, set action(A, S, C, R):")
    print("   - NOTE: Higher rank = better (more reward). Rank 3 = best, Rank 0 = worst")
    action_count = 0
    for c in range(-constants['radius'], constants['radius'] + 1):
        for r in range(-constants['radius'], constants['radius'] + 1):
            # Action 3 (right) has rank 3 (best - gives highest reward), others have rank 0 (worst - no reward)
            ctl.assign_external(Function("action", [Number(3), Number(3), Number(c), Number(r)]), True)
            for a in [0, 1, 2]:  # up, down, left
                ctl.assign_external(Function("action", [Number(a), Number(0), Number(c), Number(r)]), True)
            action_count += 4
    
    print(f"   - Total action preferences set: {action_count}")
    print("   - Action 3 (RIGHT) has rank 3 (best - highest reward) at all positions")
    print("   - Actions 0,1,2 (UP, DOWN, LEFT) have rank 0 (worst - no reward) at all positions")
    print("   - Example: action(3, 3, 0, 0) = TRUE (RIGHT is best at center)")
    print("   - Example: action(0, 0, 0, 0) = TRUE (UP is worst at center)")
    
    # Print summary
    print("\n" + "-" * 70)
    print("Input Summary:")
    print("-" * 70)
    print(f"  Constants: horizon={constants['horizon']}, obstacles={constants['obstacles']}, "
          f"radius={constants['radius']}, size={constants['size']}")
    print(f"  Target: {inputs['targets']}")
    print(f"  Walls: {len(inputs['walls'])}")
    print(f"  Restricted Areas: {len(inputs['restricted_areas'])}")
    print(f"  Obstacles: {len(inputs['obstacles'])}")
    print(f"  Multiplier: {inputs['multi']}")
    print(f"  Action Preferences: {action_count} facts")
    print("=" * 70)
    
    # Step 5: Solve
    print("\n[Step 5] Solving ASP program...")
    print("  (ASP solver is computing the optimal plan...)")
    result = ctl.solve(on_model=on_model)
    
    if result.satisfiable:
        print("\n✓ SUCCESS: Solution found!")
        print("\nExpected Result:")
        print("  - Robot should move RIGHT (action 3) to reach target at (2, 0)")
        print("  - This should happen at time step 0 (immediate next action)")
    else:
        print("\n✗ FAILURE: No solution found")
        print("This might indicate an issue with the problem setup.")


if __name__ == "__main__":
    hello_world_example()

