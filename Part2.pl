% ============================================================
% Part 2: Rescue Robot - Maximize Survivors Rescued
% Informed Search (Greedy Best-First)
% ============================================================

% ---------------------------------------------------------------
% MEMBER 1 - GRID DEFINITION
% ---------------------------------------------------------------
grid([[r, e, d, e, e],
      [e, e, f, e, s],
      [d, e, e, e, d],
      [e, s, e, f, s]]).

% ---------------------------------------------------------------
% MEMBER 1 - GRID SIZE
% ---------------------------------------------------------------
grid_size(Rows, Cols) :-
    grid(Grid),
    length(Grid, Rows),
    Grid = [FirstRow | _],
    length(FirstRow, Cols).

% ---------------------------------------------------------------
% MEMBER 1 - CELL VALUE
% ---------------------------------------------------------------
cell(Row, Col, Value) :-
    grid(Grid),
    nth1(Row, Grid, RowList),
    nth1(Col, RowList, Value).

% ---------------------------------------------------------------
% MEMBER 1 - FIND START
% ---------------------------------------------------------------
find_start(pos(Row, Col)) :-
    cell(Row, Col, r).

% ---------------------------------------------------------------
% MEMBER 1 - ALL SURVIVORS
% ---------------------------------------------------------------
all_survivors(Survivors) :-
    findall(pos(R, C), cell(R, C, s), Survivors).

% ---------------------------------------------------------------
% MEMBER 1 - MOVEMENT DIRECTIONS
% ---------------------------------------------------------------
move_r(down,    1,  0).
move_r(right,   0,  1).
move_r(up,     -1,  0).
move_r(left,    0, -1).

% ---------------------------------------------------------------
% MEMBER 1 - PASSABLE
% ---------------------------------------------------------------
passable(Row, Col) :-
    grid_size(MaxRow, MaxCol),
    Row >= 1, Row =< MaxRow,
    Col >= 1, Col =< MaxCol,
    cell(Row, Col, V),
    V \= d,
    V \= f.

% ---------------------------------------------------------------
% MEMBER 3 - STATE REPRESENTATION:
%
% A state is represented as:
%   state(CurrentPos, Visited, RescuedSurvivors)
%
% Where:
%   - CurrentPos       = pos(Row, Col)
%                        The robot's current position in the grid.
%
%   - Visited          = [pos(R1,C1), pos(R2,C2), ...]
%                        Ordered list of ALL cells visited so far
%                        (including the start cell). Used to prevent
%                        the robot from revisiting any cell in the
%                        same path. This is PER-PATH only — not
%                        global. Different search branches can visit
%                        the same cells to reach different survivors.
%
%   - RescuedSurvivors = [pos(Ra,Ca), pos(Rb,Cb), ...]
%                        List of survivor positions (cells marked s)
%                        that the robot has stepped on and rescued.
%                        This list grows each time the robot enters
%                        an s cell.
% ---------------------------------------------------------------

% ---------------------------------------------------------------
% MEMBER 3 - REACHABILITY CHECK VIA BFS
% reachable_bfs(Queue, Visited, Goal)
% Checks if Goal is reachable without revisiting cells in Visited.
% ---------------------------------------------------------------
%BASE CASE:  Goal is at the front of the queue .
bfs_check([Goal|_], _, Goal) :- !.

%RECURSIVE CASE : take the first pos(Row,Column) from the queue and expand it.
bfs_check([pos(R,C)|Rest], Visited, Goal) :-
    findall(pos(NewR,NewC),
        (move_r(_, MovedR, MovedC),
         NewR is R + MovedR,
         NewC is C + MovedC,
         passable(NewR, NewC),
         \+ member(pos(NewR,NewC), Visited)),
        Children),
    append(Rest, Children, NewQueue),
    append(Visited, Children, NewVisited ),
    bfs_check(NewQueue, NewVisited , Goal).
 
% check if Goal is reachable from the Start.
reachable(Start, Goal, Visited) :-
    bfs_check([Start], Visited, Goal).
 
% ---------------------------------------------------------------
% MEMBER 3 - HEURISTIC FUNCTION
%
% heuristic(+State, -H)
%
% Counts survivors not yet visited AND still reachable, negated.
% Greedy Best-First picks LOWEST H, so more negative = better.
%
% JUSTIFICATION:
%   - Directly measures what we want to maximise: reachable survivors ahead.
%   - Ignores survivors that are cut off, giving more accurate guidance.
%   - Guides the robot toward survivor-rich areas.
%   - Greedy Best-First does not guarantee the optimal solution
%     (the assignment explicitly allows this), but this heuristic
%     gives it strong, sensible guidance.
% ---------------------------------------------------------------
heuristic(state(pos(R,C), Visited, _), H) :-
    all_survivors(AllSurvivors),
    findall(S,
        (member(S, AllSurvivors),
         \+ member(S, Visited),
         reachable(pos(R,C), S, Visited)),
        Reachable),
    length(Reachable, Count),
    H is -Count.
% ---------------------------------------------------------------
% MEMBER 4 - GENERATE NEIGHBORS
% Revisits are blocked using per-path Visited list only.
% No global closed list — different paths must be free to
% visit the same cells to reach different survivors.
% ---------------------------------------------------------------
generate_neighbors(
    state(pos(R, C), Visited, Rescued),
    Neighbors
) :-
    findall(
        state(pos(R2, C2), NewVisited, NewRescued),
        (
            move_r(_, DR, DC),
            R2 is R + DR,
            C2 is C + DC,
            passable(R2, C2),
            \+ member(pos(R2, C2), Visited),
            append(Visited, [pos(R2, C2)], NewVisited),
            (
                cell(R2, C2, s),
                \+ member(pos(R2, C2), Rescued)
            ->
                append(Rescued, [pos(R2, C2)], NewRescued)
            ;
                NewRescued = Rescued
            )
        ),
        Neighbors
    ).

% ---------------------------------------------------------------
% MEMBER 4 - INSERT SORTED BY HEURISTIC
% ---------------------------------------------------------------
insert_sorted(Node, [], [Node]).

insert_sorted(Node, [H | T], [Node, H | T]) :-
    heuristic(Node, HN),
    heuristic(H, HH),
    HN < HH, !.

insert_sorted(Node, [H | T], [H | Rest]) :-
    insert_sorted(Node, T, Rest).

% ---------------------------------------------------------------
% MEMBER 4 - INSERT ALL CHILDREN
% ---------------------------------------------------------------
insert_all([], Open, Open).

insert_all([H | T], Open, NewOpen) :-
    insert_sorted(H, Open, Temp),
    insert_all(T, Temp, NewOpen).

% ---------------------------------------------------------------
% MEMBER 4 - GREEDY BEST-FIRST SEARCH
% Each path already prevents revisits via its own Visited list
% inside the state tuple. A global closed list was blocking
% different paths from visiting the same cells, causing the
% robot to only rescue 1 survivor instead of 2.
% ---------------------------------------------------------------

% Base case: open list empty — return best solution found.
gbfs_search([], Best, Best).

% Recursive case: expand front node (lowest H is always first).
gbfs_search([Current | Rest], CurrentBest, BestFinal) :-
    Current = state(_, _, Rescued),
    % Update best if current path rescued more survivors.
    (
        CurrentBest = none
    ->
        NewBest = Current
    ;
        CurrentBest = state(_, _, BestRescued),
        length(Rescued, C1),
        length(BestRescued, C2),
        (C1 > C2 -> NewBest = Current ; NewBest = CurrentBest)
    ),
    % Generate children using only per-path Visited.
    generate_neighbors(Current, Children),
    % Insert children sorted by heuristic.
    insert_all(Children, Rest, NewOpen),
    % Continue search.
    gbfs_search(NewOpen, NewBest, BestFinal).

% ---------------------------------------------------------------
% MEMBER 3 - MAIN OUTPUT PREDICATE
% ---------------------------------------------------------------
rescue_max_survivors :-
    find_start(pos(R0, C0)),
    (cell(R0, C0, s) -> InitRescued = [pos(R0,C0)] ; InitRescued = []),
    InitState = state(pos(R0,C0), [pos(R0,C0)], InitRescued),
    gbfs_search([InitState], none, Best),
    (
        Best = none
    ->
        write('No path found.'), nl
    ;
        Best = state(_, Path, Rescued),
        length(Path, L),
        Steps is L - 1,
        length(Rescued, NumRescued),
        write('Path found: '),
        print_path(Path), nl,
        format('Survivors rescued: ~w~n', [NumRescued]),
        format('Number of steps: ~w~n', [Steps])
    ).

% ---------------------------------------------------------------
% MEMBER 3 - PATH FORMATTING
% ---------------------------------------------------------------
print_path([pos(R, C)]) :-
    format('(~w,~w)', [R, C]).

print_path([pos(R, C) | Rest]) :-
    format('(~w,~w) -> ', [R, C]),
    print_path(Rest).
