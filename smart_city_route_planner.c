/*
 * Smart City Route Planner - Shortest-Path Suite
 * ----------------------------------------------
 * Subject : Analysis of Algorithms (AOA) - Lab ESE Mini Project
 *
 * Syllabus coverage
 *   Chapter 3 (Greedy)  : Dijkstra's algorithm
 *   Chapter 4 (DP)      : Bellman-Ford, Floyd-Warshall,
 *                         Difference between Greedy and Dynamic Programming
 *
 * Idea
 *   A city is a DIRECTED weighted graph (locations = vertices, roads = edges,
 *   weight = travel cost). The program finds cheapest routes with three
 *   algorithms, shows how each one works step by step, and checks that they
 *   agree. A dedicated demo shows WHY the greedy method (Dijkstra) fails when a
 *   road has a negative cost (a rebate / credit) while the DP methods still work.
 *
 * Compile : gcc -Wall -Wextra -O2 -o route_planner smart_city_route_planner.c
 * Run     : ./route_planner
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAXV    12                  /* max locations                         */
#define MAXE    (MAXV * MAXV)       /* max directed roads                    */
#define NAMELEN 24
#define INF     1000000000          /* "no road" / "unreachable"             */
#define MIN_W   (-100)              /* allowed road cost range               */
#define MAX_W   1000

typedef struct { int u, v, w; } Edge;

typedef struct {
    int  n, m;                      /* locations, directed roads             */
    char name[MAXV][NAMELEN];
    int  w[MAXV][MAXV];             /* adjacency matrix (INF = no road)      */
    Edge e[MAXE];                   /* edge list (same roads, for Bellman-Ford) */
} Graph;

typedef struct {                    /* result of a single-source algorithm   */
    int  dist[MAXV];
    int  parent[MAXV];
    long ops;                       /* comparisons performed                 */
    int  negCycle;                  /* Bellman-Ford: negative cycle reachable */
} SSResult;

typedef struct {                    /* result of Floyd-Warshall              */
    int  d[MAXV][MAXV];
    int  next[MAXV][MAXV];          /* next hop on the shortest i -> j path  */
    long ops;
    int  negCycle;                  /* any negative cycle in the graph       */
} APResult;

typedef enum {
    ROAD_ADDED, ROAD_UPDATED, ROAD_SELF_LOOP,
    ROAD_NEG_TWOWAY, ROAD_BAD_NODE, ROAD_BAD_WEIGHT
} RoadStatus;

/* ------------------------------------------------------------------ */
/*  Input helpers                                                      */
/* ------------------------------------------------------------------ */

static void readLine(const char *prompt, char *buf, size_t size) {
    printf("%s", prompt);
    if (!fgets(buf, (int)size, stdin)) exit(0);          /* EOF -> quit */
    buf[strcspn(buf, "\n")] = '\0';
}

static int readInt(const char *prompt, int min, int max) {
    char buf[64], *end;
    for (;;) {
        readLine(prompt, buf, sizeof buf);
        long v = strtol(buf, &end, 10);
        if (end == buf || *end != '\0') { printf("  Please enter a whole number.\n"); continue; }
        if (v < min || v > max)         { printf("  Value must be between %d and %d.\n", min, max); continue; }
        return (int)v;
    }
}

/* Print a distance; INF is shown as the word INF. */
static const char *fmt(int d) {
    static char buf[4][16];
    static int k = 0;
    k = (k + 1) % 4;
    if (d >= INF) snprintf(buf[k], sizeof buf[k], "INF");
    else          snprintf(buf[k], sizeof buf[k], "%d", d);
    return buf[k];
}

/* ------------------------------------------------------------------ */
/*  Graph operations                                                   */
/* ------------------------------------------------------------------ */

static void graphInit(Graph *g) {
    g->n = 0;
    g->m = 0;
    for (int i = 0; i < MAXV; i++)
        for (int j = 0; j < MAXV; j++) g->w[i][j] = INF;
}

/* Returns new index, -1 if full, -2 if the name already exists. */
static int addLocation(Graph *g, const char *name) {
    if (g->n >= MAXV) return -1;
    for (int i = 0; i < g->n; i++)
        if (strcmp(g->name[i], name) == 0) return -2;
    snprintf(g->name[g->n], NAMELEN, "%s", name);
    return g->n++;
}

/* Insert or update ONE directed road. */
static RoadStatus setEdge(Graph *g, int u, int v, int w) {
    if (g->w[u][v] != INF) {
        g->w[u][v] = w;
        for (int i = 0; i < g->m; i++)
            if (g->e[i].u == u && g->e[i].v == v) { g->e[i].w = w; break; }
        return ROAD_UPDATED;
    }
    g->w[u][v] = w;
    g->e[g->m].u = u;
    g->e[g->m].v = v;
    g->e[g->m].w = w;
    g->m++;
    return ROAD_ADDED;
}

static RoadStatus addRoad(Graph *g, int u, int v, int w, int twoWay) {
    if (u < 0 || u >= g->n || v < 0 || v >= g->n) return ROAD_BAD_NODE;
    if (u == v)                                   return ROAD_SELF_LOOP;
    if (w < MIN_W || w > MAX_W)                   return ROAD_BAD_WEIGHT;
    /* A two-way road with negative cost is itself a negative cycle u->v->u. */
    if (twoWay && w < 0)                          return ROAD_NEG_TWOWAY;
    RoadStatus s = setEdge(g, u, v, w);
    if (twoWay) setEdge(g, v, u, w);
    return s;
}

static int hasNegativeEdge(const Graph *g) {
    for (int i = 0; i < g->m; i++)
        if (g->e[i].w < 0) return 1;
    return 0;
}

static void loadSample(Graph *g) {
    static const char *places[] = {
        "VIT Wadala", "Sion", "Dadar", "Kurla", "Chembur", "Bandra", "Andheri", "Thane"
    };
    /* u, v, minutes, two-way? (illustrative values, not real timings) */
    static const int roads[][4] = {
        {0, 1,  8, 1}, {0, 2, 12, 1}, {1, 2,  9, 1}, {1, 3, 10, 1},
        {2, 5, 18, 1}, {3, 4, 11, 1}, {3, 6, 25, 1}, {5, 6, 14, 1},
        {3, 7, 30, 1}, {4, 7, 28, 1},
        {6, 7, 38, 0}                       /* one-way road Andheri -> Thane */
    };
    graphInit(g);
    for (size_t i = 0; i < sizeof places / sizeof places[0]; i++) addLocation(g, places[i]);
    for (size_t i = 0; i < sizeof roads / sizeof roads[0]; i++)
        addRoad(g, roads[i][0], roads[i][1], roads[i][2], roads[i][3]);
}

/* ------------------------------------------------------------------ */
/*  Display                                                            */
/* ------------------------------------------------------------------ */

static void printLegend(const Graph *g) {
    printf("\n  Locations:\n");
    for (int i = 0; i < g->n; i++) printf("    [%d] %s\n", i, g->name[i]);
}

static void printGraph(const Graph *g) {
    if (g->n == 0) { printf("\n  (graph is empty - load the sample or add locations)\n"); return; }
    printLegend(g);
    printf("\n  Roads (directed, cost):\n");
    if (g->m == 0) printf("    (none)\n");
    for (int i = 0; i < g->m; i++)
        printf("    %-*s -> %-*s  cost %d\n", NAMELEN, g->name[g->e[i].u], NAMELEN, g->name[g->e[i].v], g->e[i].w);

    printf("\n  Adjacency matrix ('-' = no direct road):\n       ");
    for (int j = 0; j < g->n; j++) printf("%5d", j);
    printf("\n");
    for (int i = 0; i < g->n; i++) {
        printf("  %4d ", i);
        for (int j = 0; j < g->n; j++) {
            if (i == j)                 printf("%5d", 0);
            else if (g->w[i][j] == INF) printf("%5s", "-");
            else                        printf("%5d", g->w[i][j]);
        }
        printf("\n");
    }
    printf("\n  Locations: %d   Roads: %d   Negative-cost road present: %s\n",
           g->n, g->m, hasNegativeEdge(g) ? "YES" : "no");
}

static void printPath(const Graph *g, const int *path, int len) {
    for (int i = 0; i < len; i++) printf("%s%s", i ? " -> " : "", g->name[path[i]]);
}

/* Rebuild src -> dst from the parent[] array. Returns length, 0 if unreachable. */
static int ssPath(const SSResult *r, int src, int dst, int path[]) {
    if (r->dist[dst] == INF) return 0;
    int tmp[MAXV], len = 0, cur = dst;
    while (cur != -1 && len < MAXV) {
        tmp[len++] = cur;
        if (cur == src) break;
        cur = r->parent[cur];
    }
    for (int i = 0; i < len; i++) path[i] = tmp[len - 1 - i];
    return len;
}

/* Rebuild i -> j from the Floyd-Warshall next[][] matrix. */
static int fwPath(const APResult *r, int u, int v, int path[]) {
    if (r->d[u][v] >= INF) return 0;
    int len = 0;
    path[len++] = u;
    while (u != v && len < MAXV) {
        u = r->next[u][v];
        path[len++] = u;
    }
    return len;
}

static void printRoutes(const Graph *g, const SSResult *r, int src) {
    printf("\n  Cheapest routes from %s\n", g->name[src]);
    printf("  %-*s %-10s %s\n", NAMELEN, "Destination", "Distance", "Route");
    for (int v = 0; v < g->n; v++) {
        int path[MAXV];
        int len = ssPath(r, src, v, path);
        printf("  %-*s %-10s ", NAMELEN, g->name[v], fmt(r->dist[v]));
        if (len) printPath(g, path, len); else printf("(unreachable)");
        printf("\n");
    }
}

/* ------------------------------------------------------------------ */
/*  Algorithm 1: Dijkstra  (GREEDY)  - O(V^2) with adjacency matrix     */
/*  Greedy choice: finalise the unvisited vertex with the smallest      */
/*  tentative distance; it can never be improved later - but ONLY if    */
/*  no edge is negative.                                                */
/* ------------------------------------------------------------------ */
static void dijkstra(const Graph *g, int src, int verbose, SSResult *r) {
    int visited[MAXV] = {0};
    r->ops = 0;
    r->negCycle = 0;
    for (int i = 0; i < g->n; i++) { r->dist[i] = INF; r->parent[i] = -1; }
    r->dist[src] = 0;

    for (int round = 1; round <= g->n; round++) {
        /* 1. greedy choice: closest unvisited vertex */
        int u = -1;
        for (int v = 0; v < g->n; v++) {
            if (visited[v]) continue;
            r->ops++;
            if (r->dist[v] != INF && (u == -1 || r->dist[v] < r->dist[u])) u = v;
        }
        if (u == -1) break;                    /* the rest are unreachable */
        visited[u] = 1;
        if (verbose) printf("  Round %d: pick %s (dist = %d)\n", round, g->name[u], r->dist[u]);

        /* 2. relax every road leaving u */
        for (int v = 0; v < g->n; v++) {
            if (visited[v] || g->w[u][v] == INF) continue;
            r->ops++;
            int nd = r->dist[u] + g->w[u][v];
            if (nd < r->dist[v]) {
                if (verbose)
                    printf("      relax %-*s %s -> %d   (via %s)\n", NAMELEN, g->name[v],
                           fmt(r->dist[v]), nd, g->name[u]);
                r->dist[v] = nd;
                r->parent[v] = u;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Algorithm 2: Bellman-Ford  (DYNAMIC PROGRAMMING) - O(V*E)           */
/*  After pass i, dist[] is correct for every route using <= i roads.   */
/*  A shortest route has at most V-1 roads, so V-1 passes suffice.      */
/*  If pass V still improves something -> negative cycle.               */
/* ------------------------------------------------------------------ */
static void bellmanFord(const Graph *g, int src, int verbose, SSResult *r) {
    r->ops = 0;
    r->negCycle = 0;
    for (int i = 0; i < g->n; i++) { r->dist[i] = INF; r->parent[i] = -1; }
    r->dist[src] = 0;

    for (int pass = 1; pass <= g->n - 1; pass++) {
        int changed = 0;
        if (verbose) printf("  Pass %d:\n", pass);
        for (int i = 0; i < g->m; i++) {
            int u = g->e[i].u, v = g->e[i].v, w = g->e[i].w;
            if (r->dist[u] == INF) continue;
            r->ops++;
            if (r->dist[u] + w < r->dist[v]) {
                if (verbose)
                    printf("      relax %s -> %s : %s becomes %d\n", g->name[u], g->name[v],
                           fmt(r->dist[v]), r->dist[u] + w);
                r->dist[v] = r->dist[u] + w;
                r->parent[v] = u;
                changed = 1;
            }
        }
        if (verbose) {
            printf("      dist after pass %d = [", pass);
            for (int i = 0; i < g->n; i++) printf("%s%s", i ? ", " : "", fmt(r->dist[i]));
            printf("]\n");
        }
        if (!changed) {
            if (verbose) printf("      no change in this pass -> stop early\n");
            break;
        }
    }
    /* Extra pass: any further improvement proves a reachable negative cycle. */
    for (int i = 0; i < g->m; i++) {
        int u = g->e[i].u, v = g->e[i].v, w = g->e[i].w;
        if (r->dist[u] == INF) continue;
        r->ops++;
        if (r->dist[u] + w < r->dist[v]) { r->negCycle = 1; break; }
    }
}

/* ------------------------------------------------------------------ */
/*  Algorithm 3: Floyd-Warshall  (DYNAMIC PROGRAMMING) - O(V^3)         */
/*  d_k[i][j] = cheapest i -> j route using only vertices 0..k-1 as     */
/*  intermediate stops.  d_k[i][j] = min(d_{k-1}[i][j],                 */
/*                                       d_{k-1}[i][k] + d_{k-1}[k][j]) */
/* ------------------------------------------------------------------ */
static void floydWarshall(const Graph *g, int verbose, APResult *r) {
    int n = g->n;
    r->ops = 0;
    r->negCycle = 0;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            if (i == j)                 { r->d[i][j] = 0;           r->next[i][j] = i;  }
            else if (g->w[i][j] != INF) { r->d[i][j] = g->w[i][j];  r->next[i][j] = j;  }
            else                        { r->d[i][j] = INF;         r->next[i][j] = -1; }
        }

    for (int k = 0; k < n; k++) {
        int improved = 0;
        if (verbose) printf("  k = %s (allow it as an intermediate stop)\n", g->name[k]);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) {
                r->ops++;
                if (r->d[i][k] == INF || r->d[k][j] == INF) continue;
                int through = r->d[i][k] + r->d[k][j];
                if (through < r->d[i][j]) {
                    if (verbose)
                        printf("      %s -> %s : %s becomes %d\n", g->name[i], g->name[j],
                               fmt(r->d[i][j]), through);
                    r->d[i][j] = through;
                    r->next[i][j] = r->next[i][k];
                    improved++;
                }
            }
        if (verbose && improved == 0) printf("      (no improvement)\n");

        /* A negative diagonal entry d[i][i] < 0 means a negative cycle exists.
         * Stop immediately: distances are undefined from here on, and carrying
         * on lets the values grow exponentially (integer overflow). */
        for (int i = 0; i < n; i++)
            if (r->d[i][i] < 0) { r->negCycle = 1; break; }
        if (r->negCycle) {
            if (verbose) printf("      negative cycle found (some d[i][i] < 0) -> stop\n");
            return;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Menu actions                                                       */
/* ------------------------------------------------------------------ */

static int needGraph(const Graph *g, int minNodes) {
    if (g->n < minNodes) {
        printf("\n  Need at least %d location(s). Load the sample graph or add locations first.\n", minNodes);
        return 0;
    }
    return 1;
}

static int readNode(const Graph *g, const char *prompt) {
    return readInt(prompt, 0, g->n - 1);
}

static void doAddLocation(Graph *g) {
    if (g->n >= MAXV) { printf("\n  Graph is full (%d locations).\n", MAXV); return; }
    char name[NAMELEN];
    do { readLine("  Location name: ", name, sizeof name); } while (name[0] == '\0');
    int idx = addLocation(g, name);
    if (idx == -2) printf("  A location with that name already exists.\n");
    else           printf("  Added [%d] %s\n", idx, g->name[idx]);
}

static void doAddRoad(Graph *g) {
    if (!needGraph(g, 2)) return;
    printLegend(g);
    int u = readNode(g, "  From (index): ");
    int v = readNode(g, "  To   (index): ");
    int w = readInt("  Cost (-100..1000, negative = rebate/credit): ", MIN_W, MAX_W);
    int two = readInt("  Two-way road? (1 = yes, 0 = one-way): ", 0, 1);
    switch (addRoad(g, u, v, w, two)) {
        case ROAD_ADDED:       printf("  Road added.\n"); break;
        case ROAD_UPDATED:     printf("  Road already existed - cost updated.\n"); break;
        case ROAD_SELF_LOOP:   printf("  Rejected: a road from a place to itself is meaningless.\n"); break;
        case ROAD_NEG_TWOWAY:  printf("  Rejected: a two-way road with negative cost is a negative cycle by itself.\n"
                                      "  Make it one-way, or use a non-negative cost.\n"); break;
        case ROAD_BAD_NODE:    printf("  Rejected: unknown location.\n"); break;
        case ROAD_BAD_WEIGHT:  printf("  Rejected: cost out of range.\n"); break;
    }
}

static void doDijkstra(const Graph *g) {
    if (!needGraph(g, 1)) return;
    if (hasNegativeEdge(g)) {
        printf("\n  Dijkstra needs NON-NEGATIVE road costs, but this graph has a negative road.\n"
               "  Greedy choices would be unsafe. Use Bellman-Ford (option 6) or Floyd-Warshall (option 7).\n"
               "  See option 9 for a demonstration of the failure.\n");
        return;
    }
    printLegend(g);
    int src = readNode(g, "  Source index: ");
    SSResult r;
    printf("\n  --- Dijkstra (greedy) trace ---\n");
    dijkstra(g, src, 1, &r);
    printRoutes(g, &r, src);
    printf("\n  Comparisons performed: %ld   (complexity O(V^2), V = %d)\n", r.ops, g->n);
}

static void doBellmanFord(const Graph *g) {
    if (!needGraph(g, 1)) return;
    printLegend(g);
    int src = readNode(g, "  Source index: ");
    SSResult r;
    printf("\n  --- Bellman-Ford (dynamic programming) trace ---\n");
    bellmanFord(g, src, 1, &r);
    if (r.negCycle) {
        printf("\n  NEGATIVE CYCLE reachable from %s: costs can be reduced forever,\n"
               "  so shortest routes are undefined. Remove or fix the cycle.\n", g->name[src]);
    } else {
        printRoutes(g, &r, src);
    }
    printf("\n  Comparisons performed: %ld   (complexity O(V*E), V = %d, E = %d)\n", r.ops, g->n, g->m);
}

static void printMatrixResult(const Graph *g, const APResult *r) {
    printf("\n  All-pairs cheapest distance (row = from, column = to):\n       ");
    for (int j = 0; j < g->n; j++) printf("%6d", j);
    printf("\n");
    for (int i = 0; i < g->n; i++) {
        printf("  %4d ", i);
        for (int j = 0; j < g->n; j++) printf("%6s", fmt(r->d[i][j]));
        printf("\n");
    }
}

static void doFloydWarshall(const Graph *g) {
    if (!needGraph(g, 1)) return;
    APResult r;
    printf("\n  --- Floyd-Warshall (dynamic programming) trace ---\n");
    floydWarshall(g, 1, &r);
    printf("\n  Comparisons performed: %ld   (complexity O(V^3), V = %d)\n", r.ops, g->n);
    if (r.negCycle) {
        printf("\n  NEGATIVE CYCLE detected (a diagonal entry d[i][i] < 0). Distances are undefined.\n");
        return;
    }
    printLegend(g);
    printMatrixResult(g, &r);
    printf("\n  Show one route:\n");
    int s = readNode(g, "  Source index: ");
    int t = readNode(g, "  Destination index: ");
    int path[MAXV];
    int len = fwPath(&r, s, t, path);
    printf("\n  %s -> %s : distance %s\n  Route: ", g->name[s], g->name[t], fmt(r.d[s][t]));
    if (len) printPath(g, path, len); else printf("(unreachable)");
    printf("\n");
}

static void doCompare(const Graph *g) {
    if (!needGraph(g, 1)) return;
    printLegend(g);
    int s = readNode(g, "  Source index: ");
    int t = readNode(g, "  Destination index: ");

    SSResult dj, bf;
    APResult fw;
    int djOK = !hasNegativeEdge(g);
    if (djOK) dijkstra(g, s, 0, &dj);
    bellmanFord(g, s, 0, &bf);
    floydWarshall(g, 0, &fw);

    printf("\n  Route query: %s -> %s   (V = %d, E = %d)\n", g->name[s], g->name[t], g->n, g->m);
    if (bf.negCycle || fw.negCycle) {
        printf("\n  Negative cycle in the graph - shortest routes are undefined. Bellman-Ford: %s, Floyd-Warshall: %s.\n",
               bf.negCycle ? "cycle reachable from source" : "none reachable from source",
               fw.negCycle ? "cycle exists somewhere" : "none");
        return;
    }
    printf("\n  %-16s %-8s %-10s %-12s %s\n", "Algorithm", "Method", "Distance", "Comparisons", "Complexity");
    printf("  ------------------------------------------------------------------\n");
    if (djOK) printf("  %-16s %-8s %-10s %-12ld %s\n", "Dijkstra", "Greedy", fmt(dj.dist[t]), dj.ops, "O(V^2)");
    else      printf("  %-16s %-8s %-10s %-12s %s\n", "Dijkstra", "Greedy", "n/a", "-", "skipped: negative road");
    printf("  %-16s %-8s %-10s %-12ld %s\n", "Bellman-Ford", "DP", fmt(bf.dist[t]), bf.ops, "O(V*E)");
    printf("  %-16s %-8s %-10s %-12ld %s\n", "Floyd-Warshall", "DP", fmt(fw.d[s][t]), fw.ops, "O(V^3)");

    int path[MAXV], len;
    printf("\n");
    if (djOK) {
        len = ssPath(&dj, s, t, path);
        printf("  Dijkstra route       : ");
        if (len) printPath(g, path, len); else printf("(unreachable)");
        printf("\n");
    }
    len = ssPath(&bf, s, t, path);
    printf("  Bellman-Ford route   : ");
    if (len) printPath(g, path, len); else printf("(unreachable)");
    printf("\n");
    len = fwPath(&fw, s, t, path);
    printf("  Floyd-Warshall route : ");
    if (len) printPath(g, path, len); else printf("(unreachable)");
    printf("\n");

    int agree = (bf.dist[t] == fw.d[s][t]) && (!djOK || dj.dist[t] == bf.dist[t]);
    printf("\n  Distances agree: %s\n", agree ? "YES" : "NO  <-- this should never happen");
}

/* Why greedy fails with a negative road. */
static void doNegativeDemo(void) {
    Graph d;
    graphInit(&d);
    addLocation(&d, "Depot");
    addLocation(&d, "Mall");
    addLocation(&d, "Hub");
    addLocation(&d, "Customer");
    addRoad(&d, 0, 1, 4, 0);       /* Depot -> Mall      cost 4                    */
    addRoad(&d, 0, 2, 5, 0);       /* Depot -> Hub       cost 5                    */
    addRoad(&d, 2, 1, -3, 0);      /* Hub   -> Mall      cost -3 (toll rebate)     */
    addRoad(&d, 1, 3, 2, 0);       /* Mall  -> Customer  cost 2                    */

    printf("\n  Demo graph (costs; negative = rebate):\n");
    printf("      Depot -> Mall 4     Depot -> Hub 5     Hub -> Mall -3     Mall -> Customer 2\n");
    printf("  Best route to Customer: Depot -> Hub -> Mall -> Customer = 5 - 3 + 2 = 4\n");

    SSResult dj, bf;
    printf("\n  Dijkstra's trace on this graph:\n");
    dijkstra(&d, 0, 1, &dj);
    bellmanFord(&d, 0, 0, &bf);

    printf("\n  %-12s %-12s %-14s %s\n", "Location", "Dijkstra", "Bellman-Ford", "Verdict");
    for (int v = 0; v < d.n; v++)
        printf("  %-12s %-12s %-14s %s\n", d.name[v], fmt(dj.dist[v]), fmt(bf.dist[v]),
               dj.dist[v] == bf.dist[v] ? "same" : "Dijkstra is WRONG");

    printf("\n  WHY: Dijkstra finalised Mall at 4 and never looked at it again (greedy).\n"
           "  Later it reached Hub, but the -3 road would have lowered Mall to 2.\n"
           "  Bellman-Ford keeps re-checking every road, so it finds the correct answer.\n"
           "  Greedy needs 'a settled vertex can never improve' - false with negative roads.\n");
}

/* ------------------------------------------------------------------ */
/*  Built-in tests                                                     */
/* ------------------------------------------------------------------ */

static int passed, total;

static void check(const char *label, int ok) {
    total++;
    if (ok) passed++;
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", label);
}

static void buildGraph(Graph *g, int n, const int edges[][3], int m) {
    char nm[NAMELEN];
    graphInit(g);
    for (int i = 0; i < n; i++) { snprintf(nm, sizeof nm, "N%d", i); addLocation(g, nm); }
    for (int i = 0; i < m; i++) setEdge(g, edges[i][0], edges[i][1], edges[i][2]);
}

static int qDij(const Graph *g, int s, int t) { SSResult r; dijkstra(g, s, 0, &r);     return r.dist[t]; }
static int qBF (const Graph *g, int s, int t) { SSResult r; bellmanFord(g, s, 0, &r); return r.dist[t]; }
static int qFW (const Graph *g, int s, int t) { APResult r; floydWarshall(g, 0, &r);  return r.d[s][t]; }

static int allAgree(const Graph *g, int s, int t, int expected) {
    return qDij(g, s, t) == expected && qBF(g, s, t) == expected && qFW(g, s, t) == expected;
}

static int allPairsAgree(const Graph *g) {
    for (int s = 0; s < g->n; s++)
        for (int t = 0; t < g->n; t++) {
            int a = qDij(g, s, t);
            if (a != qBF(g, s, t) || a != qFW(g, s, t)) return 0;
        }
    return 1;
}

static void runTests(void) {
    Graph g;
    passed = total = 0;
    printf("\n  --- Built-in test cases ---\n");

    loadSample(&g);
    check("Sample city: VIT Wadala -> Thane = 48 (all three algorithms)", allAgree(&g, 0, 7, 48));
    {   SSResult r; int p[MAXV];
        dijkstra(&g, 0, 0, &r);
        int len = ssPath(&r, 0, 7, p);
        check("Sample city: route is VIT Wadala, Sion, Kurla, Thane",
              len == 4 && p[0] == 0 && p[1] == 1 && p[2] == 3 && p[3] == 7);
    }
    check("Close alternatives: VIT Wadala -> Andheri = 43 (not 44)", allAgree(&g, 0, 6, 43));
    check("One-way road: Andheri -> Thane = 38 but Thane -> Andheri = 55",
          allAgree(&g, 6, 7, 38) && allAgree(&g, 7, 6, 55));
    check("Sample city: all 64 source/destination pairs agree", allPairsAgree(&g));
    check("Dijkstra guard: sample has no negative road", !hasNegativeEdge(&g));
    check("Source equals destination: distance 0", allAgree(&g, 3, 3, 0));

    {   static const int e[][3] = {{0,1,5},{2,3,7}};
        buildGraph(&g, 4, e, 2);
        check("Unreachable (two components): distance INF in all three", allAgree(&g, 0, 3, INF)); }

    buildGraph(&g, 1, NULL, 0);
    check("Single location, no roads: distance 0", allAgree(&g, 0, 0, 0));

    {   static const int e[][3] = {{0,1,0},{1,0,0},{1,2,5}};
        buildGraph(&g, 3, e, 3);
        check("Zero-cost roads (including a zero-cost cycle) terminate: 0 -> 2 = 5", allAgree(&g, 0, 2, 5)); }

    {   static const int e[][3] = {{0,1,1},{0,2,1},{1,3,1},{2,3,1}};
        buildGraph(&g, 4, e, 4);
        check("Two equally short routes: distance 2", allAgree(&g, 0, 3, 2)); }

    {   static const int e[][3] = {{0,1,2},{1,2,2},{2,3,2},{3,4,2},{4,5,2},{5,6,2},{6,7,2}};
        buildGraph(&g, 8, e, 7);
        check("Chain of 8 locations: forward 14, backward INF", allAgree(&g, 0, 7, 14) && allAgree(&g, 7, 0, INF)); }

    {   graphInit(&g);
        char nm[NAMELEN];
        for (int i = 0; i < 5; i++) { snprintf(nm, sizeof nm, "N%d", i); addLocation(&g, nm); }
        for (int i = 0; i < 5; i++) for (int j = 0; j < 5; j++) if (i != j) setEdge(&g, i, j, (i * 7 + j * 3) % 11 + 1);
        check("Complete graph K5 (20 roads): all 25 pairs agree", allPairsAgree(&g)); }

    {   static const int e[][3] = {{0,1,4},{0,2,5},{2,1,-3},{1,3,2}};
        buildGraph(&g, 4, e, 4);
        check("Negative road: Bellman-Ford = 4 and Floyd-Warshall = 4 (correct)", qBF(&g, 0, 3) == 4 && qFW(&g, 0, 3) == 4);
        check("Negative road: Dijkstra gives 6 (WRONG - greedy failure, as expected)", qDij(&g, 0, 3) == 6);
        check("Negative road is detected by hasNegativeEdge", hasNegativeEdge(&g)); }

    {   static const int e[][3] = {{0,1,1},{1,2,-1},{2,0,-1}};
        buildGraph(&g, 3, e, 3);
        SSResult b; APResult f;
        bellmanFord(&g, 0, 0, &b); floydWarshall(&g, 0, &f);
        check("Negative cycle reachable: Bellman-Ford and Floyd-Warshall both detect it", b.negCycle && f.negCycle); }

    {   static const int e[][3] = {{0,1,1},{1,2,-1},{2,0,-1},{2,3,4}};
        buildGraph(&g, 4, e, 4);
        SSResult b; APResult f;
        bellmanFord(&g, 3, 0, &b); floydWarshall(&g, 0, &f);
        check("Negative cycle NOT reachable from source 3: Bellman-Ford clear, Floyd-Warshall still flags graph",
              !b.negCycle && f.negCycle); }

    graphInit(&g);
    addLocation(&g, "A"); addLocation(&g, "B");
    {   addRoad(&g, 0, 1, 9, 0);
        RoadStatus s = addRoad(&g, 0, 1, 3, 0);
        check("Re-adding a road updates its cost (1 road, cost 3)", s == ROAD_UPDATED && g.m == 1 && g.w[0][1] == 3); }
    check("Input rule: self-loop rejected", addRoad(&g, 0, 0, 1, 0) == ROAD_SELF_LOOP);
    check("Input rule: two-way negative road rejected", addRoad(&g, 0, 1, -2, 1) == ROAD_NEG_TWOWAY);
    check("Input rule: cost out of range rejected", addRoad(&g, 0, 1, 5000, 0) == ROAD_BAD_WEIGHT);
    check("Input rule: duplicate location name rejected", addLocation(&g, "A") == -2);

    printf("\n  %d / %d tests passed.\n", passed, total);
}

/* ------------------------------------------------------------------ */

static void printMenu(void) {
    printf("\n==============================================\n");
    printf("   SMART CITY ROUTE PLANNER  (AOA Mini Project)\n");
    printf("==============================================\n");
    printf("   1. Load sample city graph (8 locations)\n");
    printf("   2. Add a location\n");
    printf("   3. Add a road (one-way / two-way)\n");
    printf("   4. Display graph\n");
    printf("   5. Dijkstra          - Greedy   (with trace)\n");
    printf("   6. Bellman-Ford      - DP       (with trace)\n");
    printf("   7. Floyd-Warshall    - DP       (with trace)\n");
    printf("   8. Compare all three on one route\n");
    printf("   9. Demo: why Dijkstra fails on a negative road\n");
    printf("  10. Run built-in test cases\n");
    printf("  11. Clear graph\n");
    printf("   0. Exit\n");
}

int main(void) {
    static Graph g;
    graphInit(&g);
    for (;;) {
        printMenu();
        switch (readInt("Choice: ", 0, 11)) {
            case 1:  loadSample(&g); printf("\n  Sample loaded: %d locations, %d roads (one-way road: Andheri -> Thane).\n", g.n, g.m); break;
            case 2:  doAddLocation(&g);   break;
            case 3:  doAddRoad(&g);       break;
            case 4:  printGraph(&g);      break;
            case 5:  doDijkstra(&g);      break;
            case 6:  doBellmanFord(&g);   break;
            case 7:  doFloydWarshall(&g); break;
            case 8:  doCompare(&g);       break;
            case 9:  doNegativeDemo();    break;
            case 10: runTests();          break;
            case 11: graphInit(&g); printf("\n  Graph cleared.\n"); break;
            case 0:  printf("\n  Goodbye!\n"); return 0;
        }
    }
}
