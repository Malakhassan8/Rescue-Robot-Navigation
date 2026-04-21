% ============================================================
% Part 1: Rescue Robot - Reach Nearest Survivor
% Uninformed Search (BFS)
% ============================================================

% ---------------------------------------------------------------
% MEMBER 1 - GRID DEFINITION
% ---------------------------------------------------------------
grid([[r, e, d, e, e],
      [e, e, f, e, s],
      [d, e, e, e, e],
      [e, s, e, f, e]]).

% ---------------------------------------------------------------
% MEMBER 1 - GRID SIZE
% Counts rows and columns to check boundaries.
% ---------------------------------------------------------------
grid_size(Rows, Cols) :-
    grid(Grid),
    length(Grid, Rows),
    Grid = [FirstRow | _],
    length(FirstRow, Cols).

% ---------------------------------------------------------------
% MEMBER 1 - CELL VALUE
% Gets the value of a cell at (Row, Col).
% ---------------------------------------------------------------
cell(Row, Col, Value) :-
    grid(Grid),
    nth1(Row, Grid, RowList),
    nth1(Col, RowList, Value).

% ---------------------------------------------------------------
% MEMBER 1 - FIND START
% Finds the cell containing r (robot start position).
% ---------------------------------------------------------------
find_start(pos(Row, Col)) :-
    cell(Row, Col, r).

% ---------------------------------------------------------------
% MEMBER 1 - MOVEMENT DIRECTIONS
% Four directions only, no diagonals.
% ---------------------------------------------------------------
move_r(up,     -1,  0).
move_r(down,    1,  0).
move_r(left,    0, -1).
move_r(right,   0,  1).

% ---------------------------------------------------------------
% MEMBER 1 - PASSABLE
% A cell is passable if it is inside the grid and not d or f.
% ---------------------------------------------------------------
passable(Row, Col) :-
    grid_size(MaxRow, MaxCol),
    Row >= 1, Row =< MaxRow,
    Col >= 1, Col =< MaxCol,
    cell(Row, Col, V),
    V \= d,
    V \= f.

% ---------------------------------------------------------------
% MEMBER 2 - GOAL TEST
% True if the robot is on a survivor cell.
% ---------------------------------------------------------------
goal_state(pos(Row, Col)) :-
    cell(Row, Col, s).

% ---------------------------------------------------------------
% MEMBER 2 - GENERATE NEIGHBORS
% Finds all valid next states from the current state.
% MEMBER 3 battery logic is included here (-10 per step).
% ---------------------------------------------------------------
generate_neighbors(state(pos(R, C), Path, Battery),
                   Closed,
                   Neighbors) :-
    findall(
        state(pos(R2, C2), NewPath, NewBattery),
        (
            move_r(_, DR, DC),
            R2 is R + DR,
            C2 is C + DC,
            passable(R2, C2),
            \+ member(pos(R2, C2), Closed),
            \+ member(pos(R2, C2), Path),
            % MEMBER 3 - Battery: subtract 10, block if below 0
            NewBattery is Battery - 10,
            NewBattery >= 0,
            append(Path, [pos(R2, C2)], NewPath)
        ),
        Neighbors
    ).

% ---------------------------------------------------------------
% MEMBER 2 - BFS SEARCH
% Uses explicit open list (queue) and closed list.
% Children added to BACK of queue = BFS behaviour.
% ---------------------------------------------------------------

% Base case: front of queue is a goal state.
bfs([state(pos(R, C), Path, Battery) | _], _, state(pos(R, C), Path, Battery)) :-
    goal_state(pos(R, C)), !.

% Recursive case: expand front, add children to back.
bfs([Current | RestOpen], Closed, Solution) :-
    Current = state(pos(R, C), _, _),
    generate_neighbors(Current, Closed, Children),
    append(RestOpen, Children, NewOpen),
    append(Closed, [pos(R, C)], NewClosed),
    bfs(NewOpen, NewClosed, Solution).

% No path case.
bfs([], _, no_path).

% ---------------------------------------------------------------
% MEMBER 3 - PATH FORMATTING
% Prints path as (R,C) -> (R,C) -> ... -> (R,C)
% ---------------------------------------------------------------
print_path([pos(R, C)]) :-
    format('(~w,~w)', [R, C]).
print_path([pos(R, C) | Rest]) :-
    format('(~w,~w) -> ', [R, C]),
    print_path(Rest).

% ---------------------------------------------------------------
% MEMBER 3 - MAIN OUTPUT PREDICATE
% Prints: path, number of steps, remaining battery.
% ---------------------------------------------------------------
rescue_nearest_survivor :-
    find_start(StartPos),
    InitialState = state(StartPos, [StartPos], 100),
    bfs([InitialState], [], Result),
    (
        Result = no_path
    ->
        write('No path found.'), nl
    ;
        Result = state(_, Path, Battery),
        length(Path, L),
        Steps is L - 1,
        write('Path found: '),
        print_path(Path), nl,
        format('Number of steps: ~w~n', [Steps]),
        format('Remaining Battery: ~w%~n', [Battery])
    ).
