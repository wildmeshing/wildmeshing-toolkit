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

    def valid_indices(self):
        return self.__valid_indices__
    # Check if a tuple is valid, i.e. if the simplex of dimension d-1 is a face of a simplex of dimension d
    def valid_tuple(self,t):
        items = tuple(self[i][t[i]] for i in range(len(t)))
        return all(a.issubset(b) for a,b in zip(items[:-1],items[1:]))

    def valid_tuple_as_simplicial_set(self,t):
        assert(self.valid_tuple(t))
        items = tuple(self[i][t[i]] for i in range(len(t)))
        x = (next(iter(items[0])),) + tuple(next(iter((b-a))) for a,b in zip(items[:-1],items[1:]))

        n = len(self)
        return x + (int((n * (n+1)) / 2) - sum(x),)

    def simplicial_set_as_valid_tuple(self, ss):
            
            

        return tuple(self.__simplices__[d].index(frozenset(ss[:d+1]))
                     for d in range(len(ss)-1))

    def simplicial_set_as_valid_tuple_index(self,ss):
        tup = self.simplicial_set_as_valid_tuple(ss)
        return self.valid_tuples().index(tup)


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


    def index_switch(self,tuple_index, d):
        return self.get_index(self.switch(self.tuple_from_index(tuple_index), d))
    def valid_index_switch(self,tuple_index, d):
        return self.__valid_index_map__[
            self.index_switch(
        self.__valid_indices__[tuple_index], d)
        ]

    def valid_tuples(self):
        ts = self.all_tuples()
        return [
                ts[j] for j in self.__valid_indices__
                ]

    def valid_tuple_product(self, t1, t2):
        s1 = self.valid_tuple_as_simplicial_set(t1)
        s2 = self.valid_tuple_as_simplicial_set(t2)

        sp = tuple(s1[i] for i in s2)

        return self.simplicial_set_as_valid_tuple(sp)

    def valid_tuple_index_product(self, i1, i2):
        vt = self.valid_tuples()
        return vt.index(self.valid_tuple_product(
                vt[i1],
                vt[i2]
                ))


def valid_switch_table(sc):
    return [[sc.valid_index_switch(i,d) for d in range(len(sc))] for i in range(sc.valid_tuple_size())]


def valid_switch_product_table(sc):
    size = sc.valid_tuple_size()
    return [[sc.valid_tuple_index_product(i1,i2) for i2 in range(size)] for i1 in range(size)]

def valid_switch_inverse_table(sc):
    table = valid_switch_product_table(sc)
    size = sc.valid_tuple_size()
    identity_valid_index = sc.simplicial_set_as_valid_tuple_index(tuple(range(len(sc)+1)))
    return [table[i].index(identity_valid_index) for i in range(size)] 


def switches_plus_identity(sc):
    identity = tuple(range(len(sc)+1))
    print("Identity: ", identity)
    def s(i):
        x = list(identity)
        print("1:",x)
        x[i],x[i+1] = x[i+1],x[i]
        print("2:",x)
        return tuple(x)
    sss = tuple(s(i) for i in range(len(sc))) + (identity,)
    print(sss)
    return tuple(map(sc.simplicial_set_as_valid_tuple_index,sss))

    