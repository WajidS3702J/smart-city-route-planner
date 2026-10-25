# Smart City Route Planner — Shortest-Path Suite

**Subject:** Analysis of Algorithms (AOA) — Lab ESE Mini Project
**Language:** C (console application, single file, no external libraries)

A city is modelled as a **directed, weighted graph**: locations are vertices, roads are edges, and the weight is the travel cost. The program finds the cheapest route using **three syllabus algorithms**, shows each one working step by step, checks that they agree, and demonstrates **why the greedy method (Dijkstra) breaks on negative road costs while the dynamic-programming methods (Bellman-Ford, Floyd-Warshall) do not.**

---

## 1. Syllabus coverage

| Syllabus chapter | Item | Where in this project |
|---|---|---|
| Ch 3 – Greedy Methods | Dijkstra's algorithm | `dijkstra()` – menu option 5 |
| Ch 4 – Dynamic Programming | Difference between Greedy and DP | Option 9 (live demo) + comparison in option 8 |
| Ch 4 – Dynamic Programming | Bellman-Ford algorithm | `bellmanFord()` – menu option 6 |
| Ch 4 – Dynamic Programming | Floyd-Warshall algorithm | `floydWarshall()` – menu option 7 |
| Ch 1 – Introduction | Asymptotic notation, complexity analysis | Complexity table (section 4) and operation counters in the output |

---

## 2. Problem statement

> Given a road network of locations and one-way or two-way roads with costs, find the cheapest route between locations. Support (a) one source to all destinations, (b) all pairs of locations, (c) roads with negative cost (a rebate/credit), and (d) detect the situation where a cheapest route does not exist because of a negative cycle.

**Inputs:** locations, roads (from, to, cost, one-way/two-way), source and destination.
**Outputs:** cheapest distance, the actual route, and the number of comparisons each algorithm performed.

---

## 3. Approach

**Data structures.** The graph is kept as both an **adjacency matrix** (used by Dijkstra and Floyd-Warshall) and an **edge list** (used by Bellman-Ford, so its running time is truly O(V·E)).

| Algorithm | Paradigm | Core idea |
|---|---|---|
| **Dijkstra** | Greedy | Repeatedly *finalise* the unvisited location with the smallest tentative distance, then relax its outgoing roads. A finalised location is never revisited. Valid only when **no road is negative**. |
| **Bellman-Ford** | Dynamic programming | Relax **every** road, repeat V−1 times. After pass *i*, distances are correct for all routes using at most *i* roads. One extra pass that still improves something proves a **negative cycle**. Stops early when a pass changes nothing. |
| **Floyd-Warshall** | Dynamic programming | `d[i][j] = min( d[i][j], d[i][k] + d[k][j] )` for k = 1..V — "allow location *k* as an intermediate stop". Gives **all pairs** at once. A negative value on the diagonal reveals a negative cycle. |

**Safety rules built in**
- Dijkstra is **refused** automatically if any road is negative (it would give wrong answers).
- A **two-way road with negative cost** is rejected (it is a negative cycle by itself).
- Self-loops, out-of-range costs, duplicate location names and invalid menu input are rejected.
- Re-adding an existing road **updates** its cost instead of creating a duplicate.
- `INF` arithmetic is guarded, so "unreachable" never overflows or corrupts a distance.
- Floyd-Warshall **stops as soon as a negative cycle appears** (a negative diagonal entry), because otherwise its values can grow exponentially and overflow an `int`.

---

## 4. Complexity

| Algorithm | Time | Space | Notes |
|---|---|---|---|
| Dijkstra (array / matrix version, used here) | **O(V²)** | O(V) | V rounds × (V scan + V relax) |
| Dijkstra (binary-heap version) | O((V+E) log V) | O(V) | Better for large sparse graphs |
| Bellman-Ford | **O(V·E)** | O(V) | Handles negative edges; detects negative cycles |
| Floyd-Warshall | **O(V³)** | O(V²) | All pairs; three nested loops |

For the sample graph (V = 8, E = 21) the program reports measured comparisons, e.g. for VIT Wadala → Thane:
Dijkstra **47**, Bellman-Ford **63**, Floyd-Warshall **512** (= 8³ exactly).

### Greedy vs Dynamic Programming (syllabus Ch 4, item 1)

| | Greedy (Dijkstra) | Dynamic Programming (Bellman-Ford, Floyd-Warshall) |
|---|---|---|
| Decision | Commits to the locally best choice and never reconsiders | Builds the answer from solved subproblems; keeps re-checking |
| Correct when | Edge costs are non-negative | Negative edges are fine (but no negative cycle) |
| Speed | Faster | Slower |
| Here | Fails on the option-9 demo | Gets the correct answer |

---

## 5. How to compile and run

```bash
gcc -Wall -Wextra -O2 -o route_planner smart_city_route_planner.c
./route_planner              # Linux / macOS / WSL
route_planner.exe            # Windows (MinGW / MSYS2 cmd)
.\route_planner.exe          # Windows PowerShell
```

### Menu

```
 1. Load sample city graph (8 locations)
 2. Add a location
 3. Add a road (one-way / two-way)
 4. Display graph
 5. Dijkstra          - Greedy   (with trace)
 6. Bellman-Ford      - DP       (with trace)
 7. Floyd-Warshall    - DP       (with trace)
 8. Compare all three on one route
 9. Demo: why Dijkstra fails on a negative road
10. Run built-in test cases
11. Clear graph
 0. Exit
```

**Quick start:** `1` (load sample) → `5` → source `0` to watch Dijkstra's trace. Then `8` → source `0`, destination `7` to compare all three. Then `9` for the negative-road demo.

---

## 6. Sample output

**Compare all three — VIT Wadala → Thane (option 8):**

```
  Algorithm        Method   Distance   Comparisons  Complexity
  ------------------------------------------------------------------
  Dijkstra         Greedy   48         47           O(V^2)
  Bellman-Ford     DP       48         63           O(V*E)
  Floyd-Warshall   DP       48         512          O(V^3)

  Dijkstra route       : VIT Wadala -> Sion -> Kurla -> Thane
  Distances agree: YES
```

**Why greedy fails (option 9):** Depot→Mall 4, Depot→Hub 5, Hub→Mall −3 (rebate), Mall→Customer 2.

```
  Location     Dijkstra     Bellman-Ford   Verdict
  Depot        0            0              same
  Mall         4            2              Dijkstra is WRONG
  Hub          5            5              same
  Customer     6            4              Dijkstra is WRONG
```

Dijkstra finalised *Mall* at 4 before it ever saw the −3 road; Bellman-Ford keeps re-checking every road and finds the true cost, 4.

---

## 7. Built-in tests (menu option 10 — 23 checks)

| # | Scenario | What it proves |
|---|---|---|
| 1–3 | Sample city routes (48, route shape, 43 vs 44 near-tie) | Correct distances and path reconstruction |
| 4 | One-way road: Andheri→Thane = 38, Thane→Andheri = 55 | Directed graphs handled correctly |
| 5 | All 64 source/destination pairs: Dijkstra = Bellman-Ford = Floyd-Warshall | The three algorithms agree |
| 6 | Sample has no negative road | Dijkstra guard works |
| 7 | Source = destination → 0 | Trivial case |
| 8 | Two disconnected components → INF | Unreachable handling |
| 9 | Single location, no roads | Smallest graph |
| 10 | Zero-cost roads and zero-cost cycle | Terminates, no infinite loop |
| 11 | Two equally short routes | Ties |
| 12 | Chain of 8: forward 14, backward INF | One-way chain |
| 13 | Complete graph K5 (20 roads) | Dense graph, all pairs agree |
| 14–16 | Negative road: Bellman-Ford 4, Floyd-Warshall 4, Dijkstra 6 (wrong) | Greedy vs DP |
| 17 | Negative cycle reachable | Detected by both DP algorithms |
| 18 | Negative cycle *not* reachable from source | Bellman-Ford clear, Floyd-Warshall flags the graph |
| 19 | Re-adding a road | Cost updated, no duplicate |
| 20–23 | Self-loop, negative two-way road, cost out of range, duplicate name | Input validation |

**Independent verification.** Besides the built-in tests, the algorithms were checked against a separate brute-force solver (enumerating every simple path) on **6,000 random graphs / 101,500 source-destination pairs** — covering non-negative costs, negative costs, zero costs and negative cycles — with **0 mismatches**, including validity and cost of every reconstructed route. The checker was itself confirmed to catch deliberately broken code.

---

## 8. Where this is used in industry

- **Road and delivery routing** — real engines use speed-ups of Dijkstra such as A\* and contraction hierarchies; live traffic changes the edge weights.
- **Internet routing** — OSPF computes shortest paths with Dijkstra (link-state); RIP uses a Bellman-Ford-style distance-vector method.
- **Finance** — currency arbitrage is a negative cycle (using −log of exchange rates), found with Bellman-Ford.
- **All-pairs tables** — Floyd-Warshall suits small dense networks and transitive-closure style problems.
- **Relation to NP-hardness** — finding the cheapest route to one place is in **P**; visiting *every* place once (Travelling Salesperson, Ch 4/6) is NP-hard.

## 9. Limitations and future work

- Up to 12 locations and integer costs from −100 to 1000.
- Dijkstra uses the O(V²) array version; a binary-heap version would be faster on large sparse graphs.
- A\* search, live traffic updates and file import/export of graphs are natural extensions.
- Sample travel times are illustrative, not real measurements.

## 10. Project structure

```
.
├── smart_city_route_planner.c   # complete source code
└── README.md                    # this file
```

## 11. Team

| Name | Role |
|---|---|
| _Your name_ | _e.g., Implementation_ |
| _Teammate name_ | _e.g., Testing & documentation_ |
