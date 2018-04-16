class StartingLeft():
    def __init__(self):
        self._startingPosition = "Left"

    def getStartingPosition(self):
        return self._startingPosition


class StartingMiddle():
    def __init__(self):
        self._startingPosition = "Middle"

    def getStartingPosition(self):
        return self._startingPosition


class StartingRight():
    def __init__(self):
        self._startingPosition = "Right"

    def getStartingPosition(self):
        return self._startingPosition


class ScoreScale():
    def __init__(self):
        self._scoringElement = "Scale"

    def getScoringElement(self):
        return self._scoringElement


class ScoreSwitch():
    def __init__(self):
        self._scoringElement = "Switch"

    def getScoringElement(self):
        return self._scoringElement


class CrossFieldEnable():
    def __init__(self):
        self._crossFieldEnable = True

    def getCrossFieldEnable(self):
        return self._crossFieldEnable


class CrossFieldDisable():
    def __init__(self):
        self._crossFieldEnable = False

    def getCrossFieldEnable(self):
        return self._crossFieldEnable
