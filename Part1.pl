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
