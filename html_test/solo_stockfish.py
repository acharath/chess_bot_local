import subprocess

# Path to the Stockfish executable (update this to your system's Stockfish path)
STOCKFISH_PATH = "/Users/jacobschuster/Downloads/chess_vision 134/Stockfish/src/stockfish"

# Example FEN position (random)
FEN_POSITION = "r1bqkbnr/1ppp1ppp/p1n5/4p3/2B1P3/5Q2/PPPP1PPP/RNB1K1NR w KQkq - 0 4"

def query_stockfish(fen):
    """Runs Stockfish with the given FEN position and gets the best move."""
    try:
        # Start Stockfish as a subprocess
        process = subprocess.Popen(
            [STOCKFISH_PATH], 
            stdin=subprocess.PIPE, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True
        )

        # Send FEN and "go depth 20" commands
        process.stdin.write(f"position fen {fen}\n")
        process.stdin.write("go depth 20\n")
        process.stdin.flush()

        best_move = "No move found"
        while True:
            output_line = process.stdout.readline().strip()
            if output_line.startswith("bestmove"):
                best_move = output_line
                break
            elif "Segmentation fault" in output_line or "Illegal position" in output_line:
                print(f"❌ Stockfish crashed on FEN: {fen}")
                best_move = "Stockfish crashed"
                break

        process.terminate()
        return best_move

    except Exception as e:
        print(f"❌ Error running Stockfish: {e}")
        return "Error"

if __name__ == "__main__":
    print(f"\n🎯 **FEN Position:** {FEN_POSITION}\n")
    best_move = query_stockfish(FEN_POSITION)
    print(f"🤖 **Stockfish Best Move:** {best_move}\n")
