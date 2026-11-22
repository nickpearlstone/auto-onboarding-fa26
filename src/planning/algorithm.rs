use crate::shared::{
    grid::Grid,
    types::{Path, Point2, VehicleState},
};

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};

/*takes in two grid cells, determines if they are diagonal or straight paths to each other and then  returns the corresponding cost
of traveling from one to another, diagonal is 14 and straight is 10, as a shortcut, only works when cells are neighboring.
ie. if u input cells far apart, will just return 14 instead of actual directional cost*/
pub fn local_directional_cost(current: (usize, usize), neighbor: (usize, usize)) -> usize {
    if current.0 != neighbor.0 && current.1 != neighbor.1 {
        return 14;
    }
    10
}

fn get_h(goal: (usize, usize), cell: (usize, usize)) -> usize {
    let dx = (goal.0 as isize - cell.0 as isize).abs();
    let dy = (goal.1 as isize - cell.1 as isize).abs();
    let num_diagonals = dx.min(dy);
    let num_straights = dx.max(dy) - num_diagonals;
    (num_diagonals * 14 + num_straights * 10) as usize
}

/*returns the neighbors of a given cell in a grid as a vector of cells,
but also checks if the cell is valid and not obstructed*/
fn get_neighbors(cell: (i64, i64), map: &Grid, clearance: f64) -> Vec<(usize, usize)> {
    let mut neighbors: Vec<(usize, usize)> = Vec::new();
    let candidates = [
        (cell.0 + 1, cell.1),
        (cell.0, cell.1 + 1),
        (cell.0 - 1, cell.1),
        (cell.0, cell.1 - 1),
        (cell.0 - 1, cell.1 - 1),
        (cell.0 + 1, cell.1 + 1),
        (cell.0 + 1, cell.1 - 1),
        (cell.0 - 1, cell.1 + 1),
    ];

    let Some(current) = map.center(cell.0 as usize, cell.1 as usize) else {
        return Vec::new();
    };

    for &(col, row) in &candidates {
        //leave threshold as None for now to get default 128, later add as parameter
        //think this has potential to be wrong, not sure if point w/ actual global coords cell/grid coords is supposed to be used
        if row < 0 || col < 0 {
            continue;
        }
        let Some(point) = map.center(col as usize, row as usize) else {
            continue;
        };
        if map.clear(point, clearance) && map.segment_clear(current, point, clearance) {
            neighbors.push((col as usize, row as usize));
        }
    }
    neighbors
}

/* usize is used for all of these fields because all grid access should be done with positive integers,
and for simplification I will be using 14 and 10 for diagonals and straight pathing respectively,
so g and h shouldn't cause any issues.*/
#[derive(Ord, PartialOrd, Eq, PartialEq)]
struct Node {
    f: usize,
    cell: (usize, usize),
    g: usize,
}

//TODO!!! fix git history --- branches currently overlap, also look into .expect() for option stuff --- uv.lock in home directory?
//returns nothing for the ones where there is possible paths -- also ? is not the desired shortcut, only use for result, not option bc can return none randomly
/*Planning's `run`, in `src/planning/node.rs`, is the empty one.
Subscribe to `map`, keep one `Planner` alive across messages, call `planned_update`, and publish on `path`.
Publish something for every map, including a map that isn't ready and a planner that returned `None`.
Otherwise the car keeps the previous path. `Context` has the Zenoh session, `transport.key(...)`, the goal, and the vehicle.
The message types live in `shared::protocol::pb`, and `prost::Message` turns them into bytes and back.
`Context::poll` is how the loop hears that it should quit. `Context::status` is how we can tell a wiring problem from an algorithm problem. */
//do this if want to see how it is right now

#[derive(Default)]
pub struct Planner {}

impl Planner {
    /// A route from the car to `goal`, or `None` if there is no way through.
    /// `clearance_m` is the circle from `VehicleSpec::clearance_m`.
    /// `goal_tolerance_m` is how close the last point has to be to the goal.
    /// A waypoint has to pass `Grid::clear(point, clearance_m)`. A step has to pass
    /// `Grid::segment_clear` at that same radius. `blocked` is only the cell cutoff.
    /// `validate_path` returns an error, not "no path", when clearance fails.
    /// This starter always returns `None`. See Planning in `docs/project.md`.
    pub fn plan(
        &mut self,
        map: &Grid,
        state: &VehicleState,
        goal: Point2,
        clearance_m: f64,
        goal_tolerance_m: f64,
    ) -> Option<Path> {
        //setup
        let mut opened: BinaryHeap<Reverse<Node>> = BinaryHeap::new();

        let mut closed: HashSet<(usize, usize)> = HashSet::new();

        let mut cheapest_found: HashMap<(usize, usize), usize> = HashMap::new();

        let mut came_from: HashMap<(usize, usize), (usize, usize)> = HashMap::new();

        let start_cell = map.cell(state.pose.position)?;
        let goal_cell = map.cell(goal)?;

        //add start cell, but need correct priority
        //f = h+g, we have h, need distance from the start, which should be 0
        opened.push(Reverse(Node {
            f: 0,
            cell: (start_cell.0, start_cell.1),
            g: 0,
        }));

        cheapest_found.insert(start_cell, 0);

        while !opened.is_empty() {
            //push the neighbors in the grid, but in every direction, so should be like 8 neighs for each, and then check if valid
            //need to somehow add space to hold the running cost for g at each cell, would get from one of the lookup maps, maybe the cheapest_found
            let current = opened.pop()?;
            if closed.contains(&current.0.cell) {
                continue;
            }
            if map
                .center(current.0.cell.0, current.0.cell.1)?
                .distance(goal)
                <= goal_tolerance_m
            {
                let mut path: Path = Path::new();
                let mut path_current = current.0.cell;
                //these asterisks need to be taken a look at
                //keep going through the previoius nodes, until u have a path to the start node
                while path_current != start_cell {
                    path.push(map.center(path_current.0, path_current.1)?);
                    path_current = *came_from.get(&path_current)?
                }
                //add the starting cell after the loop
                path.push(map.center(start_cell.0, start_cell.1)?);
                path.reverse();
                return Some(path);
            }
            closed.insert(current.0.cell);
            let neighbors: Vec<(usize, usize)> = get_neighbors(
                (current.0.cell.0 as i64, current.0.cell.1 as i64),
                map,
                clearance_m,
            );
            let current_g = current.0.g;
            for &neighbor in &neighbors {
                let new_g = current_g + local_directional_cost(current.0.cell, neighbor);
                let stored_cost = cheapest_found.get(&neighbor).copied().unwrap_or(usize::MAX);
                if new_g < stored_cost {
                    cheapest_found.insert(neighbor, new_g);
                    if !closed.contains(&neighbor) {
                        came_from.insert(neighbor, current.0.cell);
                        opened.push(Reverse(Node {
                            f: get_h(goal_cell, neighbor) + new_g,
                            cell: neighbor,
                            g: new_g,
                        }));
                    }
                }
            }
        }
        None
    }
}
