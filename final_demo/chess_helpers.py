# chess_helpers.py

#!/usr/bin/env python3

# Helper functions that wrap the chess engine

from stockfish import StockfishException
import chess

def is_legal(stockfish, new_position):
    """
        Determine if the current board is a legal position.
        
        Inputs:
            - stockfish instance
            - FEN position
        Outputs:
            - Boolean
    """

    try:
        if not stockfish.is_fen_valid(new_position): # Illegal position
            return False
    except StockfishException: # Error in FEN position
        return False

    #in the future we need to check whether the opponent made a legal move
    #needs to take in last position too
    return True

# def check_starting_setup(fen_position):
#     start = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR"

#     stripped_fen = fen_position.strip(" ")[0]

#     if stripped_fen == start:
#         return True
#     return False
    

#ADDITION - deleted self
def is_legal_move(old_fen, new_fen):
        # Load the board from the initial FEN
        board = chess.Board(old_fen)
        
        # Iterate over all possible legal moves
        for move in board.legal_moves:
            board.push(move)
            
            board_fen_core = " ".join(board.fen().split()[:4])[:-2].strip()
            resulting_fen_core = " ".join(new_fen.split()[:4])[:-2].strip()


            if board_fen_core.strip() == resulting_fen_core.strip():
                board.pop()
                return move.uci()
            
            board.pop()
    
        return None

#ADDITION
def check_if_game_over(fen_position):
    board = chess.Board(fen_position)
    
    # check for checkmate
    if board.is_checkmate():
        winner = "black" if board.turn == chess.WHITE else "white"
        return {
            "is_game_over": True,
            "reason": "checkmate",
            "winner": winner
        }
    
    # check for stalemate
    if board.is_stalemate():
        return {
            "is_game_over": True,
            "reason": "stalemate",
            "winner": None
        }
    
    return {
        "is_game_over": False,
        "reason": None,
        "winner": None
    }




def find_best_move(stockfish, fen_position):
    """
        Find the next best move for a chess position

        Inputs:
            - stockfish instance
            - FEN position
        Outputs:
            - Tuple
                - Next move
                - Corresponding pieces
    """
    
    try:
        stockfish.set_fen_position(str(fen_position))
        best_move = stockfish.get_best_move()
    except StockfishException:
        return None
    
    if best_move == None:
        return None
    
    #move to send back in array form
    move_to_send = []

    #start by adding standard move
    first_square = best_move[:2]
    second_square = best_move[2:4]
    move_to_send.append((first_square, second_square))

    #next add capture movement
    type_of_capture = stockfish.will_move_be_a_capture(best_move)
    if type_of_capture == stockfish.Capture.DIRECT_CAPTURE:
        move_to_send = [(second_square, "capture")] + move_to_send
    elif type_of_capture == stockfish.Capture.EN_PASSANT:
        if second_square[1] == "6":
            move_to_send = [(str(second_square[0]) + "5", "capture")] + move_to_send
        else:
            move_to_send = [(str(second_square[0]) + "4", "capture")] + move_to_send

    #promotion
    if len(best_move) == 5:
        #move pawn off board
        move_to_send.append((str(second_square), "capture"))

        #move promoted piece on baord
        piece_to_promote = best_move[-1]
        move_to_send.append((str(piece_to_promote) + "_promote", second_square))
        # move_to_send.append((str(second_square), str(piece_to_promote) + "_promote"))
    
    #ADDITION
    #check if castle
    castle_status = ""
    if best_move == "e1g1":
        move_to_send.append(("h1", "f1"))
        castle_status = ("K")
    elif best_move == "e8g8":
        move_to_send.append(("h8", "f8"))
        castle_status = ("k")
    elif best_move == "e1c1":
        move_to_send.append(("a1", "d1"))
        castle_status = ("Q")
    elif best_move == "e8c8":
        move_to_send.append(("a8", "d8"))
        castle_status = ("q")

    piece_types = []

    for start_pos, end_pos in move_to_send:
        piece_type = stockfish.get_what_is_on_square(start_pos)
        
        piece_types.append(piece_type)
    
    stockfish.make_moves_from_current_position([best_move])
    new_fen_position = stockfish.get_fen_position()



    return (move_to_send, piece_types, castle_status, new_fen_position)


