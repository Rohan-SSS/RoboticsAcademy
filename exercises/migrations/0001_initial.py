# Generated by Django 4.1.7 on 2025-02-19 10:53

import django.contrib.postgres.fields
from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):

    initial = True

    dependencies = []

    operations = [
        migrations.CreateModel(
            name="Robot",
            fields=[
                (
                    "id",
                    models.BigAutoField(
                        auto_created=True,
                        primary_key=True,
                        serialize=False,
                        verbose_name="ID",
                    ),
                ),
                ("name", models.CharField(max_length=100, unique=True)),
                ("launch_file_path", models.CharField(max_length=200)),
            ],
            options={
                "db_table": '"robots"',
            },
        ),
        migrations.CreateModel(
            name="World",
            fields=[
                (
                    "id",
                    models.BigAutoField(
                        auto_created=True,
                        primary_key=True,
                        serialize=False,
                        verbose_name="ID",
                    ),
                ),
                ("name", models.CharField(max_length=100, unique=True)),
                ("launch_file_path", models.CharField(max_length=200)),
                ("visualization_config_path", models.CharField(max_length=200)),
                (
                    "ros_version",
                    models.CharField(
                        choices=[("ROS", "ROS"), ("ROS2", "ROS2")],
                        default="none",
                        max_length=4,
                    ),
                ),
                (
                    "visualization",
                    models.CharField(
                        choices=[
                            ("none", "None"),
                            ("console", "Console"),
                            ("gazebo_gra", "Gazebo GRA"),
                            ("gazebo_rae", "Gazebo RAE"),
                            ("gzsim_gra", "Gz Sim GRA"),
                            ("gzsim_rae", "Gz Sim RAE"),
                            ("physic_gra", "Physic GRA"),
                            ("physic_rae", "Physic RAE"),
                        ],
                        default="none",
                        max_length=50,
                    ),
                ),
                (
                    "world",
                    models.CharField(
                        choices=[
                            ("none", "None"),
                            ("gazebo", "Gazebo"),
                            ("drones", "Gazebo Drones"),
                            ("gzsimdrones", "Gz Sim Drones"),
                            ("physical", "Physical"),
                        ],
                        default="none",
                        max_length=50,
                    ),
                ),
                (
                    "start_pose",
                    django.contrib.postgres.fields.ArrayField(
                        base_field=django.contrib.postgres.fields.ArrayField(
                            base_field=models.DecimalField(
                                decimal_places=4, default=None, max_digits=10
                            ),
                            size=None,
                        ),
                        size=None,
                    ),
                ),
            ],
            options={
                "db_table": '"worlds"',
            },
        ),
        migrations.CreateModel(
            name="Universe",
            fields=[
                (
                    "id",
                    models.BigAutoField(
                        auto_created=True,
                        primary_key=True,
                        serialize=False,
                        verbose_name="ID",
                    ),
                ),
                ("name", models.CharField(max_length=100, unique=True)),
                (
                    "robot",
                    models.OneToOneField(
                        db_column='"robot_id"',
                        default=None,
                        on_delete=django.db.models.deletion.CASCADE,
                        to="exercises.robot",
                    ),
                ),
                (
                    "world",
                    models.OneToOneField(
                        db_column='"world_id"',
                        default=None,
                        on_delete=django.db.models.deletion.CASCADE,
                        to="exercises.world",
                    ),
                ),
            ],
            options={
                "db_table": '"universes"',
            },
        ),
        migrations.CreateModel(
            name="Exercise",
            fields=[
                (
                    "id",
                    models.BigAutoField(
                        auto_created=True,
                        primary_key=True,
                        serialize=False,
                        verbose_name="ID",
                    ),
                ),
                ("exercise_id", models.CharField(max_length=40, unique=True)),
                ("name", models.CharField(max_length=40, unique=True)),
                ("description", models.CharField(max_length=400)),
                ("tags", models.CharField(default='{"tags": ""}', max_length=2000)),
                (
                    "status",
                    models.CharField(
                        choices=[
                            ("ACTIVE", "ACTIVE"),
                            ("INACTIVE", "INACTIVE"),
                            ("PROTOTYPE", "PROTOTYPE"),
                        ],
                        default="ACTIVE",
                        max_length=20,
                    ),
                ),
                ("template", models.CharField(blank=True, default="", max_length=200)),
                (
                    "universes",
                    models.ManyToManyField(
                        db_table='"exercises_universes"',
                        default=None,
                        to="exercises.universe",
                    ),
                ),
            ],
            options={
                "db_table": '"exercises"',
            },
        ),
    ]
