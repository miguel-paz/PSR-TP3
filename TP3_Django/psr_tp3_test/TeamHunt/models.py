from django.db import models

# Create your models here.

class Coach(models.Model):
    name = models.CharField(max_length=200, primary_key=True)

    def __str__(self):
        return self.name


class Robot(models.Model):
    name = models.CharField(max_length=200, primary_key=True)
    state = models.CharField(max_length=200)
    coach = models.ForeignKey(Coach, on_delete=models.CASCADE)
    def __str__(self):
        return self.name

class Message(models.Model):
    id = models.BigAutoField(primary_key=True)
    sender = models.ForeignKey(Robot, on_delete=models.CASCADE)
    previous_state = models.CharField(max_length=200)
    current_state = models.CharField(max_length=200)
    reason = models.CharField(max_length=200)
    receiver = models.ForeignKey(Coach, on_delete=models.CASCADE)

    def __str__(self):
        return str(self.id)
