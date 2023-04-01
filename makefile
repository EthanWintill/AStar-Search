CC=g++
CPPFLAGS= -std=c++11 -Wall -I $(INCDIR)
EXEC=Project2-edw53
DB=debug

OBJDIR=.#/obj
OBJ=$(addprefix $(OBJDIR)/,$(EXEC).o)
DBOBJ=$(addprefix $(OBJDIR)/,$(DB).o)

INCDIR=.#/inc
INC=$(addprefix $(INCDIR)/,)

SRCDIR=.#/src
SRC=$(addprefix $(SRCDIR)/,$(EXEC).cpp)


$(EXEC): $(OBJ)
	$(CC) -o $@ $^ $(CPPFLAGS)

$(DB): $(DBOBJ)
	$(CC) -g -o $@ $^ $(CPPFLAGS)

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp $(addprefix $(INCDIR)/,%.h)
	$(CC) -c -o $@ $< $(CPPFLAGS)

$(OBJDIR)/$(EXEC).o: $(SRCDIR)/$(EXEC).cpp $(INC)
	$(CC) -c -o $@ $< $(CPPFLAGS)

$(OBJDIR)/%DB.o: $(SRCDIR)/%.cpp $(addprefix $(INCDIR)/,%.h)
	$(CC) -c -g -o $@ $< $(CPPFLAGS)

$(OBJDIR)/$(DB).o: $(SRCDIR)/$(EXEC).cpp $(INC)
	$(CC) -c -g -o $@ $< $(CPPFLAGS)


.PHONY: clean run

clean:
	rm -f $(EXEC) $(OBJDIR)/*.o $(DB)

run: $(EXEC)
	./$(EXEC)
