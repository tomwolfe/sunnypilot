# Test structure
def test_function():
    try:
        print("inside try")
        x = 1
        y = 2
        return x + y
    except Exception as e:
        print(f"error: {e}")
        return 0