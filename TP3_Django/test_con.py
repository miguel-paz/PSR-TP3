import sqlite3

try:
    sqliteConnection = sqlite3.connect('db.sqlite3')
    cursor = sqliteConnection.cursor()
    print("Successfully Connected to Django DB")

    sqlite_select_Query = "select sqlite_version();"
    cursor.execute(sqlite_select_Query)

    record = cursor.fetchall()
    print("SQLite Database Version is: ", record)

    select_Robos = '''SELECT * from Robots'''
    #cursor.execute('''INSERT INTO EMPLOYEE(FIRST_NAME, LAST_NAME, AGE, SEX,
#INCOME) VALUES ('Ramya', 'Rama Priya', 27, 'F', 9000)''')
    record = cursor.fetchall()
    print("Robots: ", record)
    
    cursor.close()

except sqlite3.Error as error:
    print("Error while connecting to Django DB", error)
finally:
    if sqliteConnection:
        sqliteConnection.close()
        print("The Django DB connection is closed")