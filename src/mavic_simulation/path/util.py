def valida_float(P):
    if P == "":
        return True
    try:
        float(P)
        return True
    except ValueError:
        return False