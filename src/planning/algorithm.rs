use crate::{
    shared::grid::Grid,
    shared::types::{Path, Point2, VehicleState},
};

use std::collections::{BinaryHeap, HashMap, HashSet};
use std::cmp::Reverse;


fn get_h(goal: (usize, usize), cell: (usize, usize)) -> usize{
    let dx = (goal.0 as isize - cell.0 as isize).abs();
    let dy = (goal.1 as isize - cell.1 as isize).abs();
    let num_diagonals = dx.min(dy);
    let num_straights = dx.max(dy) - num_diagonals;
    (num_diagonals * 14 + num_straights * 10) as usize
}

/* usize is used for all of these fields because all grid access should be done with positive integers, 
    and for simplification I will be using 14 and 10 for diagonals and straight pathing respectively, 
    so g and h shouldn't cause any issues.*/
#[derive(Ord, PartialOrd, Eq, PartialEq)]
struct Node {
    f: usize,
    cell: (usize,usize),
    g: usize,
}
    
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
        let mut closed: HashSet<(usize,usize)> = HashSet::new();

        let mut cheapest_found: HashMap<(usize, usize), usize> = HashMap::new();

        let mut came_from: HashMap<(usize, usize), (usize, usize)> = HashMap::new();


        let start_cell = map.cell(state.pose.position)?;
        let goal_cell = map.cell(goal)?;
        
        //TODO: actually implement A*


    }
}
