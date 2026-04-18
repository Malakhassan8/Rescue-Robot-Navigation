% ============================================================
% Part 1: Rescue Robot - Reach Nearest Survivor
% Uninformed Search (BFS)
% ============================================================
% ---------------------------------------------------------------
% GRID DEFINITION. 
% ---------------------------------------------------------------
grid([[r, e, d, e, e],
      [e, e, f, e, s],
      [d, e, e, e, e],
      [e, s, e, f, e]]).

% ---------------------------------------------------------------
% Ensures robot stays within the boundaries. 
% ---------------------------------------------------------------
grid_size(Rows, Cols) :-
    grid(Grid),
    length(Grid, Rows),
    Grid = [FirstRow | _],
    length(FirstRow, Cols).

% ---------------------------------------------------------------
% Helper to get cell value at (Row, Col). 
% ---------------------------------------------------------------
cell(Row, Col, Value) :-
    grid(Grid),
    nth1(Row, Grid, RowList),
    nth1(Col, RowList, Value).

% ---------------------------------------------------------------
% Starting point at r. 
% ---------------------------------------------------------------
find_start(pos(Row, Col)) :-
    cell(Row, Col, r).

% ---------------------------------------------------------------
% Four directions only, no diagonals. 
% ---------------------------------------------------------------
move_r(up,    -1,  0).
move_r(down,   1,  0).
move_r(left,   0, -1).
move_r(right,  0,  1).

% ---------------------------------------------------------------
% Cannot move into Debris (d) or Fire (f), stay in boundaries. 
% ---------------------------------------------------------------
passable(Row, Col) :-
    grid_size(MaxRow, MaxCol),
    Row >= 1, Row =< MaxRow,
    Col >= 1, Col =< MaxCol,
    cell(Row, Col, V),
    V \= d,
    V \= f.


% GOAL TEST True If robot is currently on a survivor cell 

goal_state(pos(Row, Col)) :-
    cell(Row, Col, s).


% generate valid neighbors

generate_neighbors(state(pos(R,C), Path, Battery),
                   Closed,
                   Neighbors) :-

    findall(
        state(pos(R2,C2), NewPath, NewBattery),

        (
            move_r(_, DR, DC),
            R2 is R + DR,
            C2 is C + DC,
            passable(R2, C2),
            \+ member(pos(R2,C2), Closed),
            \+ member(pos(R2,C2), Path),   % extra safety (no revisit in path)

            NewBattery is Battery - 10,
            NewBattery >= 0,

            append(Path, [pos(R2,C2)], NewPath)
        ),

        Neighbors
    ).



% bfs search loop


bfs(Open, _, state(pos(R,C), Path, Battery)) :-
    Open = [state(pos(R,C), Path, Battery) | _],
    goal_state(pos(R,C)), !.


bfs(Open, Closed, Solution) :-
    Open = [Current | RestOpen],

    Current = state(pos(R,C), _, _),

    generate_neighbors(Current, Closed, Children),

    append(RestOpen, Children, NewOpen),
    append(Closed, [pos(R,C)], NewClosed),

    bfs(NewOpen, NewClosed, Solution).



% no path case_ 

bfs([], _, no_path).


% main entry predicate 

rescue_nearest_survivor :-

    find_start(StartPos),

    InitialState = state(StartPos, [StartPos], 100),

    bfs([InitialState], [], Result),

    (
        Result = no_path ->
            write('No path found.'), nl

        ;

        Result = state(_, Path, Battery),

        length(Path, L),
        Steps is L - 1,

        write('Path found: '),
        print_path(Path), nl,

        write('Number of steps: '),
        write(Steps), nl,

        write('Remaining Battery: '),
        write(Battery), write('%'), nl
    ).




% helper to print path 

print_path([pos(R,C)]) :-
    format('(~w,~w)', [R,C]).

print_path([pos(R,C)|Rest]) :-
    format('(~w,~w) -> ', [R,C]),
    print_path(Rest).
