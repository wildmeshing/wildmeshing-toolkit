from itertools import product

class SimplexComplex:
    def __init__(self, simplices):
        self.__simplices__ = list(map(lambda x: list(map(frozenset,x)), simplices))
        
        self.__valid_indices__ = [self.get_index(t) for t in self.all_tuples() if self.valid_tuple(t)]
        self.__valid_index_map__ = {k:v for v,k in enumerate(self.__valid_indices__)}

    def __getitem__(self, index):
        return self.__simplices__.__getitem__(index)
# Generates all possible tuples, note that some might not be valid
    def all_tuples(self):
        r = list(product(*map(list,map(range,map(len,self.__simplices__)))))
        return r

    # Check if a tuple is valid, i.e. if the simplex of dimension d-1 is a face of a simplex of dimension d
    def valid_tuple(self,t):
        items = tuple(self[i][t[i]] for i in range(len(t)))
        return all(a.issubset(b) for a,b in zip(items[:-1],items[1:]))

    def __len__(self):
        return self.__simplices__.__len__()

    # Enumerates all valid tuples similar to t, but with a different value in the slot d
    # There must be 2 of them, and this function returns the one that is different than t
    def switch(self,t,d):
        assert(self.valid_tuple(t))
        t = tuple(t)
        candidates = []
        left = t[:d]
        right = t[d+1:]
        candidates = [left + (i,) + right for i in range(len(self[d])) if i != t[d]]
        valid = [x for x in candidates if self.valid_tuple(x)]
        assert(len(valid) == 1)
        return valid[0]


    def get_index(self, t):
        assert(self.valid_tuple(t))
        return self.all_tuples().index(t)

    def tuple_from_index(self,index):
        return self.all_tuples()[index]

    def valid_tuple_index(self,index):
        return self.tuple_from_index(self.__valid_indices__[index])

    def valid_tuple_size(self):
        return len(self.__valid_indices__)

    def valid_tuple_index(self, index):
        return self.__valid_indices__[index]
