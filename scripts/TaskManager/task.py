class Task:
    """
        Task Interface

        Virtual Methods:
            getState()                  :       String State
            execute(String Command)     :       Bool Success, String Msg          
    """

    def getState():
        raise NotImplementedError()

    def execute():
        raise NotImplementedError()