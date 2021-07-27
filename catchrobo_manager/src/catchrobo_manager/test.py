
class ActionState(object):
    CALC_BISCO = 1
    GO_ABOVE_BISCO = 2
    
state = ActionState()

print(ActionState.CALC_BISCO)
print(ActionState.CALC_BISCO+1)
print(ActionState.CALC_BISCO == 1)