#include <string.h>

#include <openbmc_intf.h>
#include <openbmc.h>
#include <gpio.h>

static const gchar* dbus_object_path = "/org/openbmc/control";
static const gchar* object_name = "/org/openbmc/control/wol_control";
static const gchar* dbus_name = "org.openbmc.control.Checkstop";

static GDBusObjectManagerServer *manager = NULL;

GPIO checkwol = (GPIO){ "WOL_CONTROL" };

static gboolean
chassis_poweron(gpointer connection)
{
    GDBusProxy *proxy;
    GError *error;
    GVariant *parm = NULL;
    GVariant *result = NULL;
	
    printf("Host WOL, going to chassis_poweron\n");
    error = NULL;
    proxy = g_dbus_proxy_new_sync((GDBusConnection*)connection,
        G_DBUS_PROXY_FLAGS_NONE,
        NULL, /* GDBusInterfaceInfo* */
        "org.openbmc.control.Chassis", /* name */
        "/org/openbmc/control/chassis0", /* object path */
        "org.openbmc.control.Chassis", /* interface name */
        NULL, /* GCancellable */
        &error);
    g_assert_no_error(error);

    error = NULL;
    result = g_dbus_proxy_call_sync(proxy,
        "powerOn",
        NULL,
        G_DBUS_CALL_FLAGS_NONE,
        -1,
        NULL,
        &error);
    g_assert_no_error(error);
    g_variant_unref(result);

    return FALSE;
}


static gboolean
on_wol_interrupt(GIOChannel *channel,
        GIOCondition condition,
        gpointer connection)
{
    GError *error = 0;
    gsize bytes_read = 0;
    gchar buf[2];
	int rc = 0;
	uint8_t gpio = 0;
    buf[1] = '\0';

    g_io_channel_seek_position( channel, 0, G_SEEK_SET, 0 );
    g_io_channel_read_chars(channel,
            buf, 1,
            &bytes_read,
            &error );
	
    printf("checkstop wol: %s\n",buf);
    if(checkwol.irq_inited) {
		rc = gpio_open(&checkwol);
   		if (rc == GPIO_OK) {
        	rc = gpio_read(&checkwol, &gpio);
			if (rc == GPIO_OK && (!gpio) ) {
				g_timeout_add(1000, chassis_poweron, connection);
			}
    	}
		gpio_close(&checkwol);
    }
    else {
        checkwol.irq_inited = true;
    }

    return TRUE;
}


static void
on_bus_acquired(GDBusConnection *connection,
        const gchar *name,
        gpointer object)
{
    int rc = GPIO_OK;
    manager = g_dbus_object_manager_server_new(dbus_object_path);

    ControlCheckstop* control_checkstop = control_checkstop_skeleton_new();
    object_skeleton_set_control_checkstop(object, control_checkstop);
    g_object_unref(control_checkstop);

    g_dbus_object_manager_server_set_connection(manager, connection);

	rc = gpio_init(connection, &checkwol);
    if (rc == GPIO_OK) {
        rc = gpio_open_interrupt(&checkwol, on_wol_interrupt, connection);
    }
    if (rc != GPIO_OK) {
        printf("ERROR Checkstop: WOL GPIO setup (rc=%d)\n", rc);
    }
    g_dbus_object_manager_server_export(manager, G_DBUS_OBJECT_SKELETON(object));
}

static void
on_name_acquired(GDBusConnection *connection,
        const gchar *name,
        gpointer object)
{
}

static void
on_name_lost(GDBusConnection *connection,
        const gchar *name,
        gpointer object)
{
}

gint
main(gint argc, gchar *argv[])
{
    GMainLoop *loop;
    ObjectSkeleton *newobject;

    newobject = object_skeleton_new(object_name);

    guint id;
    loop = g_main_loop_new(NULL, FALSE);

    id = g_bus_own_name(DBUS_TYPE,
            dbus_name,
            G_BUS_NAME_OWNER_FLAGS_ALLOW_REPLACEMENT |
            G_BUS_NAME_OWNER_FLAGS_REPLACE,
            on_bus_acquired,
            on_name_acquired,
            on_name_lost,
            newobject,
            NULL);

    g_main_loop_run(loop);

    g_bus_unown_name(id);
    g_object_unref(newobject);
    g_main_loop_unref(loop);
    return 0;
}

