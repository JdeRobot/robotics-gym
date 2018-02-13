class EnvrionmentSpecs(object):
    __specs__ = {}

    def __new__(cls, *args, **kwargs):
        obj = super(EnvrionmentSpecs,cls).__new__(cls, *args, **kwargs)
        if len(cls.__specs__) == 0:
            cls.__specs__ = obj.__dict__
        else:
            obj.__dict__ = cls.__specs__
        return obj

    def __iter__(self):
        return iter(self.__specs__)
