import numpy as np
import pickle
import os
DEBUG = os.environ.get('DEBUG', 'False') == 'True'
EPISODES = 50000
from tqdm import tqdm
from random import random
from collections import deque

class Learning:

    def __init__(self, alpha=0.1, gamma=0.8, epsilon=0.6):
        # Hyperparameters
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.actions = [0, 1, 2, 3]

        self.q_table = None
        self.states = []
        self.rewards = []
        self.current = None
        self.name = None

    def create_states(self, instance):
        # build states once
        self.states = [(c, r)
                    for c in range(1, instance.size + 1)
                    for r in range(1, instance.size + 1)
                    if (c, r) not in instance.walls]
        idx = {s: i for i, s in enumerate(self.states)}      # O(1) index
        S = set(self.states)                                  # O(1) membership

        moves = {0:(0,1), 1:(0,-1), 2:(-1,0), 3:(1,0)}
        n = len(self.states)
        rewards = [None]*n

        for i, s in enumerate(self.states):
            r = {}
            for a, (dx,dy) in moves.items():
                nx, ny = s[0]+dx, s[1]+dy
                state = (nx, ny)
                next_idx = i
                done = False
                reward = -10
                if state in S:
                    next_idx = idx[state]
                    if state != s:
                        reward = -1
                    if state == instance.target:
                        reward = 100
                        done = True
                r[a] = [(1.0, next_idx, reward, done)]
            rewards[i] = r

        self.rewards = rewards
        self.current = idx[instance.robot]

    def create_starting_states(self, instance):
        walls = set(instance.walls)
        seen = set([instance.target])
        q = deque([instance.target])
        actions = [(0,1),(0,-1),(-1,0),(1,0)]
        N = instance.size

        while q:
            x, y = q.popleft()
            for dx, dy in actions:
                nx, ny = x+dx, y+dy
                if 1 <= nx <= N and 1 <= ny <= N:
                    cand = (nx, ny)
                    if cand not in seen and cand not in walls:
                        seen.add(cand)
                        q.append(cand)

        seen.remove(instance.target)
        return list(seen)

    def get_action(self, instance):
        if instance.robot == instance.target:
            return None
        state_idx = self.states.index(instance.robot)
        actions = np.copy(self.q_table[state_idx])
        actions = np.argsort(actions)

        action = actions[len(actions) - 1]
        return action

    def get_action_rank(self, robot, action):
        if robot not in self.states:
            return -1
        state_idx = self.states.index(robot)
        actions = np.copy(self.q_table[state_idx])
        actions = np.argsort(actions)
        actions = list(np.array(actions))

        return actions.index(action)


    def _build_transition_arrays(self, instance):
        # map state->index once
        idx = {s:i for i,s in enumerate(self.states)}
        self._idx = idx

        A = len(self.actions)
        S = len(self.states)

        # Dense arrays: next_state[s,a], reward[s,a], done[s,a]
        next_state = np.empty((S, A), dtype=np.int32)
        reward     = np.empty((S, A), dtype=np.float32)
        done       = np.empty((S, A), dtype=np.bool_)

        # self.rewards[s][a] = [(prob, next_idx, rew, done)]  # prob is always 1.0 here
        for s in range(S):
            rdict = self.rewards[s]
            for a in range(A):
                _, ns, rw, dn = rdict[a][0]
                next_state[s, a] = ns
                reward[s, a]     = rw
                done[s, a]       = dn

        self._T_next = next_state
        self._T_rew  = reward
        self._T_done = done

    def learn(self, instance, episodes=None, extra_rollouts_per_start=5, max_steps_per_ep=10_000):
        # 1) Build states and reachables with the O(|S|) versions from the previous reply
        self.create_states(instance)
        starting_states = self.create_starting_states(instance)

        # 2) Precompute transitions into dense arrays
        self._build_transition_arrays(instance)

        S = len(self.states)
        A = len(self.actions)
        idx = self._idx

        # starting indices as an array (no list.index in loop)
        start_indices = np.array([idx[s] for s in starting_states], dtype=np.int32)
        robot_idx = idx[instance.robot]

        # defaults
        EPISODES = episodes if episodes is not None else 5_000

        total_iters = EPISODES + len(start_indices) * extra_rollouts_per_start

        # 3) Q-table
        self.q_table = np.zeros((S, A), dtype=np.float32)

        # 4) Main loop with pure array indexing
        for i in tqdm(range(1, total_iters + 1), desc=f"Learning {instance.name}", unit="ep"):
            if i <= EPISODES:
                s = robot_idx
            else:
                s = start_indices[i % len(start_indices)]

            steps = 0
            while True:
                # epsilon-greedy without Python list ops
                if random() < self.epsilon:
                    a = np.random.randint(A)
                else:
                    a = int(np.argmax(self.q_table[s]))

                ns = int(self._T_next[s, a])
                rw = float(self._T_rew[s, a])
                dn = bool(self._T_done[s, a])

                # Q update in-place
                old = self.q_table[s, a]
                next_max = float(self.q_table[ns].max())
                self.q_table[s, a] = (1.0 - self.alpha) * old + self.alpha * (rw + self.gamma * next_max)

                s = ns
                steps += 1
                if dn or steps >= max_steps_per_ep:
                    break

        pickle.dump(self, open(f"{instance.name}.pkl", 'wb'))
