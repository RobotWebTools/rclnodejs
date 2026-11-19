# Graph Example

The [graph example](ros-graph-example.mjs) creates publishers, subscriptions, a service, and a client under `/ns1` and `/ns2`, then prints the nodes and their endpoints.

## Run

With ROS sourced and `npm install` completed, run from the repository root:

```bash
node example/graph/ros-graph-example.mjs
```

The example needs `std_msgs` and `example_interfaces`. See the [build requirements](../../docs/BUILDING.md#requirements) for checkout setup.

## Read the Output

`NODES` lists node names and their namespace information. The remaining sections show topic and service types, then the publishers, subscriptions, services, and clients associated with each node. Built-in endpoints such as `/rosout` and parameter services also appear.

`getNodeNames()` returns names without namespaces. Use `getNodeNamesAndNamespaces()` when you need both; a name alone does not distinguish nodes in different namespaces.

The script inspects the graph; it does not send messages or call the service. In particular, its relative client name resolves to `/ns2/add_two_ints`, while the service is `/ns1/add_two_ints`. A client must use that server's full name, or a remapping, to call it across namespaces.

Graph results are discovery snapshots. Other processes can change what appears, and newly created remote endpoints may take time to show up.

For the discovery methods and return types, see the [Node declarations](../../types/node.d.ts).