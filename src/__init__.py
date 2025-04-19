def input_typed(prompt: str, tp: type):
    if tp == bool:
        tp_fn = lambda x: {'y': True, 'n': False}[x]
    else:
        tp_fn = tp
    while True:
        try:
            return tp_fn(input(prompt))
        except:
            print('invalid input')