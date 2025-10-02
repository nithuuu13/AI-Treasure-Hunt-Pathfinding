from Game import Game

if __name__ == '__main__':
    game = Game(
        'c:\Users\Nithu\Downloads\AI Assignment 2\locations.txt')
    print("Node at (0, 3):", game.node_grid[1][1].content)
    print("Node at (0, ):", game.all_treasures)

    #run A* search alg 
    path, nodes_expanded = game.solve()

    #displaying results
    if path:
        print("\nPath to collectall treasures:")
        for step in path:
            print(step)
        print(f"\nTotal steps: {len(path)}")
    else:
        print("\nNo path found to collect all treasures.")

    print(f"Nodes expanded during search: {nodes_expanded}")


    # game.display_map()
    # initial_state = game.create_initial_state()
    # print("Initial State:", initial_state)
