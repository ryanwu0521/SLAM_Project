import cTheia

graph = cTheia.TraversalGraph('src/TheiaSLAM/graphs/test_world_1_graph.json')
print(graph.nodes[0].edges)